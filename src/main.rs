pub mod pregen;

use clap::{Parser, Subcommand};
use pregen::Pregen;
use rcon::Connection;
use tokio::net::TcpStream;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
#[command(propagate_version = true)]
struct GeneralArgs {
    /// Host address of the server to connect via RCON
    #[arg(short, long)]
    host: String,

    /// Port of the server to connect via rcon
    #[arg(short, long, default_value_t = 25575)]
    port: u16,

    /// Password to connect to the RCON server
    #[arg(long)]
    password: String,

    #[command(subcommand)]
    commands: Commands,
}

#[derive(Subcommand, Debug)]
enum Commands {
    /// Pregen the world given the parameters
    Pregen(Pregen),
    Interactive,
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let args = GeneralArgs::parse();

    let address = format!("{}:{}", args.host, args.port);

    let conn = Connection::<TcpStream>::builder()
        .enable_minecraft_quirks(true)
        .connect(address, &args.password)
        .await?;

    match args.commands {
        Commands::Pregen(command) => pregen::execute(command, conn).await,
        _ => todo!(),
    }
}
