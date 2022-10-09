use std::{cmp::Ordering, collections::HashSet, path::Path, time::Duration};

use anyhow::bail;
use cgmath::Point2;
use clap::{Args, ValueEnum};
use linestring::linestring_2d::{convex_hull::ConvexHull, LineString2};
use rcon::Connection;
use tokio::net::TcpStream;

const CHUNK_SIZE: isize = 16;

#[derive(Args, Debug)]
pub struct Pregen {
    /// Name of the player that will be used to pregen the world
    #[arg(long)]
    player_name: String,

    /// Pregen mode
    #[arg(long, value_enum, default_value_t = <Mode as Default>::default())]
    mode: Mode,

    /// Starting X coordinate
    #[arg(long, default_value_t = -800)]
    x0: isize,

    /// Starting Z coordinate
    #[arg(long, default_value_t = -800)]
    z0: isize,

    /// Ending X coordinate
    #[arg(long, default_value_t = 800)]
    x1: isize,

    /// Ending Z coordinate
    #[arg(long, default_value_t = 800)]
    z1: isize,

    /// Stride in the X coordinates
    #[arg(long, default_value_t = 8)]
    sx: u8,

    /// Stride in the Z coordinates
    #[arg(long, default_value_t = 8)]
    sz: u8,

    /// Dimension to generate
    #[arg(long, default_value_t = 0)]
    dimension_id: u8,

    /// Time to wait in seconds
    #[arg(long, default_value_t = 20)]
    wait: u64,
}

#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, ValueEnum, Debug)]
pub enum Mode {
    /// From the center to the sides
    Expanding,

    /// From top left to bottom right
    Filling,
}

impl Default for Mode {
    fn default() -> Self {
        Self::Filling
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Chunks {
    /// Size of the map in chunks
    x_size: usize,
    /// Size of the map in chunks
    z_size: usize,

    chunks: Vec<Vec<bool>>,
}

impl Chunks {
    pub(crate) async fn load_chunks<P: AsRef<Path>>(path: P) -> anyhow::Result<Self> {
        let data = tokio::fs::read_to_string(path).await?;
        let chunks = serde_json::from_str(&data)?;
        Ok(chunks)
    }

    /// Every coordindates is expressed in chunks
    pub(crate) fn new(x0: isize, z0: isize, x1: isize, z1: isize) -> Self {
        let chunks = (z0..z1)
            .map(|_| (x0..x1).map(|_| false).collect())
            .collect();

        let x_size = (x1 - x0) as usize;
        let z_size = (z1 - z0) as usize;

        Self {
            x_size,
            z_size,
            chunks,
        }
    }

    /// x and z are already recentered compared to the map
    pub(crate) fn fill_stride(
        &mut self,
        x: usize,
        z: usize,
        base_points: &[(isize, isize)],
    ) -> anyhow::Result<()> {
        for (i, j) in base_points {
            if let Some(row) = self.chunks.get_mut(*j as usize + z) {
                if let Some(cell) = row.get_mut(*i as usize + x) {
                    *cell = true;
                } else {
                    eprintln!(
                        "Couldn't find cell ({}, {})",
                        *i as usize + x,
                        *j as usize + z
                    )
                }
            } else {
                eprintln!("Couldn't find row {}", *j as usize + z)
            }
        }

        Ok(())
    }

    pub(crate) fn find_teleport_points(
        &self,
        offset_x: isize,
        offset_z: isize,
        sx: u8,
        sz: u8,
    ) -> Vec<(isize, isize)> {
        let horizontal_step = sx as isize;
        let vertical_step = sz as isize;

        let initial_teleport_point = (
            (horizontal_step / 2) - (offset_x),
            (vertical_step / 2) - (offset_z),
        );

        let mut first_row = vec![initial_teleport_point.clone()];
        let mut last_point = first_row.last().unwrap().clone();

        while last_point.1 <= (self.z_size as isize - offset_z) - vertical_step - 1 {
            first_row.push((last_point.0, last_point.1 + vertical_step));
            last_point = first_row.last().unwrap().clone();
        }

        let mut inter_points = vec![];
        let mut all_rows = vec![first_row];

        while last_point.0 <= (self.x_size as isize - offset_x) - horizontal_step - 1 {
            let last_row = all_rows.last().unwrap();
            let new_row = last_row
                .iter()
                .map(|(x, z)| (x + horizontal_step, *z))
                .collect::<Vec<_>>();

            let points = last_row
                .windows(2)
                .zip(new_row.windows(2))
                .map(|(previous, new)| {
                    let tl = previous[0];
                    let br = new[1];
                    find_centroid(tl, br)
                });

            inter_points.extend(points);
            last_point = new_row.last().unwrap().clone();
            all_rows.push(new_row);
        }

        let mut points = all_rows.into_iter().flatten().collect::<Vec<_>>();

        points.extend(inter_points);

        let points = points.into_iter().collect::<HashSet<_>>();

        points
            .into_iter()
            .filter(|(x, z)| {
                self.chunks[*x as usize + offset_x as usize][*z as usize + offset_z as usize]
                    == false
            })
            .collect()
    }

    pub(crate) async fn save_chunks<P: AsRef<Path>>(&self, path: P) -> anyhow::Result<()> {
        let data = serde_json::to_string(&self)?;
        std::fs::write(path, data)?;
        Ok(())
    }
}

/// Given two extreme points, give the centroid
fn find_centroid(tl: (isize, isize), br: (isize, isize)) -> (isize, isize) {
    ((br.0 + tl.0) / 2, (br.1 + tl.1) / 2)
}

pub(crate) async fn execute(
    command: Pregen,
    mut conn: Connection<TcpStream>,
) -> anyhow::Result<()> {
    println!(
        "Pregenerating from ({}, {}) to ({}, {})",
        command.x0, command.z0, command.x1, command.z1
    );

    let center_x = (command.x0 + command.x1) / 2;
    let center_z = (command.z0 + command.z1) / 2;

    println!("Determining center to be ({center_x}, {center_z})");

    let chunked_x0 = command.x0 / CHUNK_SIZE;
    let chunked_z0 = command.z0 / CHUNK_SIZE;
    let chunked_x1 = command.x1 / CHUNK_SIZE;
    let chunked_z1 = command.z1 / CHUNK_SIZE;

    println!(
        "Chunks ({}, {}) to ({}, {}) will be generated: {} chunks to generate",
        chunked_x0,
        chunked_z0,
        chunked_x1,
        chunked_z1,
        (chunked_z1 - chunked_z0) * (chunked_x1 - chunked_x0)
    );

    let mut chunks = match Chunks::load_chunks("./chunks.json").await {
        Ok(chunks) => chunks,
        Err(_) => Chunks::new(chunked_x0, chunked_z0, chunked_x1, chunked_z1),
    };

    let chunked_center_x = center_x / CHUNK_SIZE;
    let chunked_center_z = center_z / CHUNK_SIZE;

    let offset_x = chunked_center_x - chunked_x0;
    let offset_z = chunked_center_z - chunked_z0;

    let mut points = chunks.find_teleport_points(offset_x, offset_z, command.sx, command.sz);

    println!("Number of teleportation points found: {}", points.len());

    match command.mode {
        Mode::Expanding => {
            points.sort_by(|a, b| {
                let distance_a = ((a.0 - chunked_center_x).pow(2) as f64
                    + (a.1 - chunked_center_z).pow(2) as f64)
                    .sqrt();
                let distance_b = ((b.0 - chunked_center_x).pow(2) as f64
                    + (b.1 - chunked_center_z).pow(2) as f64)
                    .sqrt();

                match distance_a.total_cmp(&distance_b) {
                    o @ Ordering::Less => o,
                    o @ Ordering::Greater => o,
                    Ordering::Equal => match a.1.cmp(&b.1).reverse() {
                        o @ Ordering::Less => o,
                        o @ Ordering::Greater => o,
                        Ordering::Equal => a.0.cmp(&b.0).reverse(),
                    },
                }
            });
        }
        Mode::Filling => {
            println!("Filling mode used. No need to do aditionnal modifications to points.")
        }
    }

    let base_convex_hull = generate_convex_hull(command.sx as usize);
    let hull_points = generate_list_of_points(&base_convex_hull, command.sx as isize);

    for (x, z) in points {
        let real_x = x * CHUNK_SIZE + 8;
        let real_z = z * CHUNK_SIZE + 8;
        let cmd = format!(
            "cofh tpx {} {real_x} 320 {real_z} {}",
            command.player_name, command.dimension_id
        );
        println!("Sending: {cmd}");
        let result = conn.cmd(&cmd).await?;

        if result.starts_with("That player cannot") {
            bail!("Couldn't find player on server")
        }

        println!("Server: {result}");

        let chunked_x = if x < 0 { x - 1 } else { x };

        let chunked_z = if x < 0 { z - 1 } else { z };

        let chunked_x = (chunked_x + offset_x) as usize;
        let chunked_z = (chunked_z + offset_z) as usize;

        chunks.fill_stride(chunked_x, chunked_z, &hull_points)?;
        chunks.save_chunks("./chunks.json").await?;

        tokio::time::sleep(Duration::from_secs(command.wait)).await;
    }

    Ok(())
}

fn generate_convex_hull(stride: usize) -> LineString2<f64> {
    use line_drawing::BresenhamCircle;

    let circle = BresenhamCircle::new(0, 0, stride as isize - 1);

    let points = circle
        .map(|(x, y)| Point2::new(x as f64, y as f64))
        .collect::<Vec<_>>();

    let (left, right): (Vec<Point2<f64>>, Vec<Point2<f64>>) =
        points.into_iter().partition(|p| p.x < 0.0);

    let (mut tl, mut bl): (Vec<Point2<f64>>, Vec<Point2<f64>>) =
        left.into_iter().partition(|p| p.y > 0.0);

    let (mut tr, mut br): (Vec<Point2<f64>>, Vec<Point2<f64>>) =
        right.into_iter().partition(|p| p.y > 0.0);

    tl.sort_by(|a, b| a.x.total_cmp(&b.x).then_with(|| a.y.total_cmp(&b.y)));
    bl.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .reverse()
            .then_with(|| a.y.total_cmp(&b.y))
    });
    tr.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .then_with(|| a.y.total_cmp(&b.y).reverse())
    });
    br.sort_by(|a, b| {
        a.x.total_cmp(&b.x)
            .reverse()
            .then_with(|| a.y.total_cmp(&b.y).reverse())
    });

    let mut points = vec![];

    points.extend(tr);
    points.extend(br);
    points.extend(bl);
    points.extend(tl);

    ConvexHull::graham_scan(&points)
}

fn generate_list_of_points(hull: &LineString2<f64>, stride: isize) -> Vec<(isize, isize)> {
    let mut points = vec![];

    for j in -(stride - 1)..=(stride - 1) {
        for i in -(stride - 1)..=(stride - 1) {
            if ConvexHull::contains_point_inclusive(hull, [i as f64, j as f64].into()) {
                points.push((i, j));
            }
        }
    }

    points
}
