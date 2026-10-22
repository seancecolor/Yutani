// Copyright 2026 <Visitant Media LLC>
// Yutani - Building Better Color profiles

mod driver;
mod icc;
mod gui;

use anyhow::Result;
use clap::{Parser, Subcommand};

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
                println!("Starting CLI spot measurement & BT.1886 profile generation...");
                match driver::spyder::SpyderSession::open() {
                    Ok(session) => {
                        let reading = session.measure()?;
                        let (x, y, lum) = reading.to_xyy();
                        println!(
                            "Measured current screen: {:.3} cd/m² (CIE xy = {:.4}, {:.4})",
                            lum, x, y
                        );
                        let white_lum = if lum > 5.0 { lum } else { 100.0 };
                        icc::generate_bt1886_profile(white_lum, 0.05);
                    }
                    Err(e) => {
                        println!("Sensor unavailable ({e:#}); generating baseline 100-nit BT.1886 profile...");
                        icc::generate_bt1886_profile(100.0, 0.05);
                    }
                }
            }
        }
    } else {
        gui::run_gui().map_err(|e| anyhow::anyhow!("GUI error: {e}"))?;
    }

    Ok(())
}
