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
        /// Display model target (pa32ucdm, pa279crv, mq16ahe)
        #[arg(short, long, default_value = "pa32ucdm")]
        display: String,
        /// Target color gamut space (rec709, p3, native)
        #[arg(long, default_value = "rec709")]
        gamut: String,
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
                display,
                gamut,
                gamma,
                pure_power,
                srgb,
            } => {
                let target_display = icc::DisplayTarget::from_cli_str(&display);
                let target_gamut = icc::ColorGamut::from_cli_str(&gamut);
                let eotf = if srgb {
                    icc::EotfConfig::Srgb
                } else if pure_power {
                    icc::EotfConfig::PurePower { gamma }
                } else {
                    icc::EotfConfig::Bt1886 { gamma }
                };
                println!(
                    "Starting CLI spot measurement & profile generation for {} [{}] ({})...",
                    target_display.full_name(),
                    target_gamut.display_label(),
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
                        let white_lum = if lum > 5.0 {
                            lum
                        } else {
                            target_display.default_target_nits()
                        };
                        icc::generate_bt1886_profile_with_eotf(
                            target_display,
                            target_gamut,
                            white_lum,
                            target_display.fallback_black_nits(),
                            eotf,
                        );
                    }
                    Err(e) => {
                        println!(
                            "Sensor unavailable ({e:#}); generating baseline profile for {}...",
                            target_display.full_name()
                        );
                        icc::generate_bt1886_profile_with_eotf(
                            target_display,
                            target_gamut,
                            target_display.default_target_nits(),
                            target_display.fallback_black_nits(),
                            eotf,
                        );
                    }
                }
            }
        }
    } else {
        gui::run_gui().map_err(|e| anyhow::anyhow!("GUI error: {e}"))?;
    }

    Ok(())
}
