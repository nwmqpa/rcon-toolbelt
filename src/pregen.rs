use std::{cmp::Ordering, path::Path, time::Duration};

use anyhow::Context;
use clap::{Args, ValueEnum};
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
    pub(crate) fn fill_stride(&mut self, x: usize, z: usize, sx: u8, sz: u8) -> anyhow::Result<()> {
        let x0 = x - (sx as usize);
        let x1 = x + (sx as usize) - 1;

        let z0 = z - (sz as usize);
        let z1 = z + (sz as usize) - 1;

        for j in z0..z1 {
            let row = self
                .chunks
                .get_mut(j)
                .context(format!("Can't get row {j}"))?;

            for i in x0..x1 {
                let cell = row
                    .get_mut(i)
                    .context(format!("Can't get cell at address ({i}, {j})"))?;

                *cell = true;
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
        ((sz as usize - 1)..(self.z_size - sz as usize + 1))
            .step_by(sz as usize - 1)
            .map(|j| {
                let z = ((j as isize - offset_z) * CHUNK_SIZE) + (CHUNK_SIZE / 2);

                ((sx as usize - 1)..(self.x_size - sx as usize + 1))
                    .step_by(sx as usize - 1)
                    .map(|i| {
                        let x = ((i as isize - offset_x) * CHUNK_SIZE) + (CHUNK_SIZE / 2);
                        (x, z)
                    })
                    .collect::<Vec<(isize, isize)>>()
            })
            .flatten()
            .collect()
    }

    pub(crate) async fn save_chunks<P: AsRef<Path>>(&self, path: P) -> anyhow::Result<()> {
        let data = serde_json::to_string(&self)?;
        std::fs::write(path, data)?;
        Ok(())
    }
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
                let distance_a =
                    ((a.0 - center_x).pow(2) as f64 + (a.1 - center_z).pow(2) as f64).sqrt();
                let distance_b =
                    ((b.0 - center_x).pow(2) as f64 + (b.1 - center_z).pow(2) as f64).sqrt();

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

    for (x, z) in points {
        let cmd = format!(
            "cofh tpx {} {x} 320 {z} {}",
            command.player_name, command.dimension_id
        );
        println!("Sending: {cmd}");
        let result = conn.cmd(&cmd).await?;
        println!("Server: {result}");

        let chunked_x = if x < 0 {
            (x / CHUNK_SIZE) - 1
        } else {
            x / CHUNK_SIZE
        };

        let chunked_z = if x < 0 {
            (z / CHUNK_SIZE) - 1
        } else {
            z / CHUNK_SIZE
        };

        let chunked_x = (chunked_x + offset_x) as usize;
        let chunked_z = (chunked_z + offset_z) as usize;

        chunks.fill_stride(chunked_x, chunked_z, command.sx, command.sz)?;
        chunks.save_chunks("./chunks.json").await?;

        tokio::time::sleep(Duration::from_secs(command.wait)).await;
    }

    Ok(())
}
