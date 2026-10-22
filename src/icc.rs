use std::path::{Path, PathBuf};
use anyhow::{Context, Result};
use lcms2::{CIExyY, CIExyYTRIPLE, Profile, ToneCurve};

use crate::driver::spyder::XyzReading;

/// Electro-Optical Transfer Function (EOTF) / Gamma mode for calibration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EotfConfig {
    /// ITU-R BT.1886 black-level compensated power law (standard γ = 2.40, or custom γ).
    Bt1886 { gamma: f64 },
    /// Pure power-law gamma curve `L_norm = V^gamma` (e.g. 2.20, 2.40, 2.60).
    PurePower { gamma: f64 },
    /// IEC 61966-2-1 sRGB piecewise transfer function.
    Srgb,
}

impl Default for EotfConfig {
    fn default() -> Self {
        Self::Bt1886 { gamma: 2.4 }
    }
}

impl EotfConfig {
    pub fn gamma_value(self) -> f64 {
        match self {
            Self::Bt1886 { gamma } | Self::PurePower { gamma } => gamma,
            Self::Srgb => 2.2,
        }
    }

    pub fn display_label(self) -> String {
        match self {
            Self::Bt1886 { gamma } => {
                format!("ITU-R BT.1886 (γ = {gamma:.2} + Black Level Compensation)")
            }
            Self::PurePower { gamma } => format!("Pure Power Law (γ = {gamma:.2})"),
            Self::Srgb => String::from("IEC 61966-2-1 sRGB Piecewise (~γ 2.20)"),
        }
    }

    pub fn filename_slug(self) -> String {
        match self {
            Self::Bt1886 { gamma } => {
                if (gamma - 2.4).abs() < 0.005 {
                    String::from("Yutani_PA32UCDM_BT1886.icc")
                } else {
                    format!("Yutani_PA32UCDM_BT1886_G{gamma:.2}.icc")
                }
            }
            Self::PurePower { gamma } => format!("Yutani_PA32UCDM_Gamma_{gamma:.2}.icc"),
            Self::Srgb => String::from("Yutani_PA32UCDM_sRGB.icc"),
        }
    }
}

/// Complete set of patch measurements captured during a calibration run.
#[derive(Clone, Debug)]
pub struct CalibrationMeasurements {
    pub black: XyzReading,
    pub white: XyzReading,
    pub red: XyzReading,
    pub green: XyzReading,
    pub blue: XyzReading,
    /// `(stimulus_0_to_1, reading)` pairs across the grayscale ramp.
    pub grayscale: Vec<(f64, XyzReading)>,
}

/// Compute BT.1886 parameters `(a, b)` for a given white/black luminance (cd/m²) and gamma.
pub fn bt1886_params(white_lum: f64, black_lum: f64, gamma: f64) -> (f64, f64) {
    let g = gamma.clamp(1.0, 4.0);
    let lw = white_lum.max(1.0);
    let lb = black_lum.clamp(0.0, lw * 0.2);
    let lw_pow = lw.powf(1.0 / g);
    let lb_pow = lb.powf(1.0 / g);
    let denom = (lw_pow - lb_pow).max(1e-6);
    let a = denom.powf(g);
    let b = lb_pow / denom;
    (a, b)
}

/// Evaluate target luminance (in cd/m²) at input signal `v` in `[0.0, 1.0]` for the chosen EOTF.
pub fn eotf_target_luminance(
    v: f64,
    white_lum: f64,
    black_lum: f64,
    eotf: EotfConfig,
) -> f64 {
    let v = v.clamp(0.0, 1.0);
    let lw = white_lum.max(1.0);
    let lb = black_lum.clamp(0.0, lw * 0.2);
    match eotf {
        EotfConfig::Bt1886 { gamma } => {
            let g = gamma.clamp(1.0, 4.0);
            let (a, b) = bt1886_params(lw, lb, g);
            let v_plus_b = (v + b).max(0.0);
            a * v_plus_b.powf(g)
        }
        EotfConfig::PurePower { gamma } => {
            let g = gamma.clamp(1.0, 4.0);
            lb + (lw - lb) * v.powf(g)
        }
        EotfConfig::Srgb => {
            let lin = if v <= 0.04045 {
                v / 12.92
            } else {
                ((v + 0.055) / 1.055).powf(2.4)
            };
            lb + (lw - lb) * lin
        }
    }
}

/// Build the 1024-point 16-bit Tone Reproduction Curve for the selected EOTF.
fn build_eotf_lut(white_lum: f64, black_lum: f64, eotf: EotfConfig) -> Vec<u16> {
    let lw = white_lum.max(1.0);
    let lb = black_lum.clamp(0.0, lw * 0.2);
    let span = (lw - lb).max(1e-6);

    let mut curve_data = Vec::with_capacity(1024);
    for i in 0..1024 {
        let v = i as f64 / 1023.0;
        let l = eotf_target_luminance(v, lw, lb, eotf);
        let l_norm = (l - lb) / span;
        let u16_val = (l_norm.clamp(0.0, 1.0) * 65535.0).round() as u16;
        curve_data.push(u16_val);
    }
    curve_data
}

/// Generate a baseline ICC profile from white/black luminance values and EOTF configuration.
pub fn generate_bt1886_profile_with_eotf(white_lum: f64, black_lum: f64, eotf: EotfConfig) {
    let curve_data = build_eotf_lut(white_lum, black_lum, eotf);
    let trc_curve = ToneCurve::new_tabulated(&curve_data);
    let curves = [&trc_curve, &trc_curve, &trc_curve];

    // BT.709 primaries used as baseline when unmeasured
    let mut profile = Profile::new_rgb(
        &CIExyY { x: 0.3127, y: 0.3290, Y: 1.0 },
        &CIExyYTRIPLE {
            Red: CIExyY { x: 0.64, y: 0.33, Y: 0.2126 },
            Green: CIExyY { x: 0.30, y: 0.60, Y: 0.7152 },
            Blue: CIExyY { x: 0.15, y: 0.06, Y: 0.0722 },
        },
        &curves,
    )
    .unwrap();

    let filename = eotf.filename_slug();
    profile
        .save_profile_to_file(Path::new(&filename))
        .unwrap();
    println!("Saved {filename}");
}

/// Generate a calibrated ICC profile from real Spyder sensor measurements and chosen EOTF,
/// saving both to the current working directory and `~/.local/share/icc/`.
pub fn generate_profile_from_measurements(
    meas: &CalibrationMeasurements,
    eotf: EotfConfig,
) -> Result<Vec<PathBuf>> {
    // If white luminance is near-zero (e.g. cap was left on during a dry run),
    // fall back to sensible 100-nit / D65 / BT.709 defaults so profile creation still succeeds.
    let valid_optical = meas.white.y > 1.0 && (meas.red.y + meas.green.y + meas.blue.y) > 0.5;

    let (white_lum, black_lum) = if valid_optical {
        (meas.white.y, meas.black.y.min(meas.white.y * 0.1))
    } else {
        (100.0, 0.05)
    };

    let white_xyy = if valid_optical {
        let (wx, wy, _) = meas.white.to_xyy();
        CIExyY {
            x: wx.clamp(0.20, 0.45),
            y: wy.clamp(0.20, 0.45),
            Y: 1.0,
        }
    } else {
        CIExyY {
            x: 0.3127,
            y: 0.3290,
            Y: 1.0,
        }
    };

    let primaries = if valid_optical {
        let (rx, ry, r_lum) = meas.red.to_xyy();
        let (gx, gy, g_lum) = meas.green.to_xyy();
        let (bx, by, b_lum) = meas.blue.to_xyy();
        let lum_sum = (r_lum + g_lum + b_lum).max(1e-6);
        CIExyYTRIPLE {
            Red: CIExyY {
                x: rx.clamp(0.50, 0.75),
                y: ry.clamp(0.20, 0.45),
                Y: r_lum / lum_sum,
            },
            Green: CIExyY {
                x: gx.clamp(0.15, 0.45),
                y: gy.clamp(0.50, 0.80),
                Y: g_lum / lum_sum,
            },
            Blue: CIExyY {
                x: bx.clamp(0.10, 0.25),
                y: by.clamp(0.02, 0.20),
                Y: b_lum / lum_sum,
            },
        }
    } else {
        CIExyYTRIPLE {
            Red: CIExyY { x: 0.64, y: 0.33, Y: 0.2126 },
            Green: CIExyY { x: 0.30, y: 0.60, Y: 0.7152 },
            Blue: CIExyY { x: 0.15, y: 0.06, Y: 0.0722 },
        }
    };

    let curve_data = build_eotf_lut(white_lum, black_lum, eotf);
    let trc_curve = ToneCurve::new_tabulated(&curve_data);
    let curves = [&trc_curve, &trc_curve, &trc_curve];

    let mut profile = Profile::new_rgb(&white_xyy, &primaries, &curves)
        .context("failed to construct lcms2 RGB profile")?;

    let mut saved_paths = Vec::new();
    let filename = eotf.filename_slug();

    let local_path = PathBuf::from(&filename);
    profile
        .save_profile_to_file(&local_path)
        .with_context(|| format!("failed to write {filename}"))?;
    saved_paths.push(local_path);

    if let Ok(home) = std::env::var("HOME") {
        let icc_dir = PathBuf::from(home).join(".local/share/icc");
        if std::fs::create_dir_all(&icc_dir).is_ok() {
            let user_icc = icc_dir.join(&filename);
            if profile.save_profile_to_file(&user_icc).is_ok() {
                saved_paths.push(user_icc);
            }
        }
    }

    Ok(saved_paths)
}

