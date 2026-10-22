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
    Detect {
        #[arg(long)]
        all: bool,
    },
    Probe,
    Calibrate {
        /// Target gamma exponent (e.g. 2.4, 2.2, 2.6)
        #[arg(short, long, default_value_t = 2.4)]
        gamma: f64,
        /// Use pure power-law gamma instead of BT.1886 black-level compensation
        #[arg(long)]
        pure_power: bool,
        /// Use IEC 61966-2-1 sRGB piecewise curve
        #[arg(long)]
        srgb: bool,
    },
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
            Command::Calibrate {
                gamma,
                pure_power,
                srgb,
            } => {
                let eotf = if srgb {
                    icc::EotfConfig::Srgb
                } else if pure_power {
                    icc::EotfConfig::PurePower { gamma }
                } else {
                    icc::EotfConfig::Bt1886 { gamma }
                };
                println!(
                    "Starting CLI spot measurement & profile generation ({})...",
                    eotf.display_label()
                );
                match driver::spyder::SpyderSession::open() {
                    Ok(session) => {
                        let reading = session.measure()?;
                        let (x, y, lum) = reading.to_xyy();
                        println!(
                            "Measured current screen: {:.3} cd/m² (CIE xy = {:.4}, {:.4})",
                            lum, x, y
                        );
                        let white_lum = if lum > 5.0 { lum } else { 100.0 };
                        icc::generate_bt1886_profile_with_eotf(white_lum, 0.05, eotf);
                    }
                    Err(e) => {
                        println!("Sensor unavailable ({e:#}); generating baseline 100-nit profile...");
                        icc::generate_bt1886_profile_with_eotf(100.0, 0.05, eotf);
                    }
                }
            }
        }
    } else {
        gui::run_gui().map_err(|e| anyhow::anyhow!("GUI error: {e}"))?;
    }

    Ok(())
}
