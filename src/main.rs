// Copyright 2026 <Visitant Media LLC>
// Yutani - Building Better Color profiles

mod driver;
mod icc;
mod gui;

use anyhow::Result;
use clap::{Parser, Subcommand};
use lcms2::{Profile, ToneCurve};

#[derive(Parser)]
#[command(name = "yutani", version, about)]
struct Cli {
    #[command(subcommand)]
    command: Option<Command>,
}

#[derive(Subcommand)]
enum Command {
    Detect { #[arg(long)] all: bool },
    Probe,
    Calibrate,
}

fn main() -> Result<()> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "yutani=info".into()),
        )
        .init();

    let cli = Cli::parse();

    if let Some(cmd) = cli.command {
        match cmd {
            Command::Detect { all } => driver::detect(all)?,
            Command::Probe => driver::spyder::probe()?,
            Command::Calibrate => {
                println!("Starting calibration...");
                // Note: Implement Wayland full-screen patch display and measurement sequence
                // For demonstration, we just generate the BT.1886 curve directly.
                icc::generate_bt1886_profile(100.0, 0.05); // Example values: 100 nits white, 0.05 nits black
            }
        }
    } else {
        gui::run_gui()?;
    }

    Ok(())
}
