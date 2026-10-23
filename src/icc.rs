use std::path::{Path, PathBuf};
use anyhow::{Context, Result};
use lcms2::{
    CIExyY, CIExyYTRIPLE, Locale, Profile, Tag, TagSignature, ToneCurve, ToneCurveRef, MLU,
};

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

/// Summary of the computed 1D VCGT calibration LUT for display in the telemetry report.
#[derive(Clone, Debug)]
pub struct VcgtSummary {
    /// Whitepoint D65 channel gain scaling `[R, G, B]` in `0.0..=1.0`.
    pub d65_rgb_gains: [f64; 3],
    /// VCGT output value (normalized `0.0..=1.0`) at 50% input stimulus for `[R, G, B]`.
    pub midtone_50_vcgt: [f64; 3],
    /// Whether real optical measurements were used to invert the panel's native response.
    pub used_measured_inversion: bool,
}

/// Invert a 3x3 matrix `m`. Returns `None` if singular.
fn invert_3x3(m: [[f64; 3]; 3]) -> Option<[[f64; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if det.abs() < 1e-9 {
        return None;
    }
    let inv_det = 1.0 / det;

    let mut inv = [[0.0f64; 3]; 3];
    inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;

    inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;

    inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;

    Some(inv)
}

fn mat_vec_mul(m: [[f64; 3]; 3], v: [f64; 3]) -> [f64; 3] {
    [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
    ]
}

/// Given a monotonically non-decreasing measured curve `(u_k, y_k)` with `u_k` in `[0, 1]`
/// and `y_k` normalized to `[0, 1]`, invert `y(u) = target_y` to find the panel drive `u*`.
fn invert_measured_curve(samples: &[(f64, f64)], target_y: f64) -> f64 {
    if samples.is_empty() {
        return target_y.clamp(0.0, 1.0);
    }
    let y = target_y.clamp(0.0, 1.0);
    if y <= samples[0].1 {
        return samples[0].0;
    }
    let last = samples[samples.len() - 1];
    if y >= last.1 {
        return last.0;
    }

    for win in samples.windows(2) {
        let (u0, y0) = win[0];
        let (u1, y1) = win[1];
        if y >= y0 && y <= y1 {
            let dy = y1 - y0;
            if dy <= 1e-9 {
                return 0.5 * (u0 + u1);
            }
            let t = (y - y0) / dy;
            return (u0 + t * (u1 - u0)).clamp(0.0, 1.0);
        }
    }

    y
}

/// Build the 3-channel 1024-entry `vcgt` LUTs (`[Vec<u16>; 3]`) by decomposing the measured
/// grayscale ramp into R, G, B primary contributions, computing D65 white-balance gains,
/// and inverting each channel's measured electro-optical response against the target EOTF.
pub fn compute_vcgt_tables(
    meas: &CalibrationMeasurements,
    eotf: EotfConfig,
) -> ([Vec<u16>; 3], VcgtSummary) {
    let valid_optical = meas.white.y > 1.0
        && (meas.red.y + meas.green.y + meas.blue.y) > 0.5
        && meas.grayscale.len() >= 2;

    if !valid_optical {
        // Fallback identity linear ramp when unmeasured (e.g. dry run with cap on)
        let mut ident = Vec::with_capacity(1024);
        for i in 0..1024 {
            let v = i as f64 / 1023.0;
            ident.push((v * 65535.0).round() as u16);
        }
        return (
            [ident.clone(), ident.clone(), ident],
            VcgtSummary {
                d65_rgb_gains: [1.0, 1.0, 1.0],
                midtone_50_vcgt: [0.5, 0.5, 0.5],
                used_measured_inversion: false,
            },
        );
    }

    let bx = meas.black.x;
    let by = meas.black.y;
    let bz = meas.black.z;

    // Columns of primary matrix M_prim are the black-subtracted R, G, B primary XYZ vectors
    let m_prim = [
        [
            (meas.red.x - bx).max(0.0),
            (meas.green.x - bx).max(0.0),
            (meas.blue.x - bx).max(0.0),
        ],
        [
            (meas.red.y - by).max(0.0),
            (meas.green.y - by).max(0.0),
            (meas.blue.y - by).max(0.0),
        ],
        [
            (meas.red.z - bz).max(0.0),
            (meas.green.z - bz).max(0.0),
            (meas.blue.z - bz).max(0.0),
        ],
    ];

    let lw = meas.white.y.max(1.0);
    let lb = meas.black.y.clamp(0.0, lw * 0.2);
    let net_lw = (lw - lb).max(1e-6);

    // Compute D65 channel gains and per-channel grayscale response curves
    let (d65_gains, channel_curves) = if let Some(m_inv) = invert_3x3(m_prim) {
        // Target CIE D65 white tristimulus (x=0.3127, y=0.3290) at net peak luminance
        let d65_x = (0.3127 / 0.3290) * net_lw;
        let d65_y = net_lw;
        let d65_z = ((1.0 - 0.3127 - 0.3290) / 0.3290) * net_lw;
        let raw_d65_rgb = mat_vec_mul(m_inv, [d65_x, d65_y, d65_z]);

        // Also compute native white RGB in primary space
        let native_w_rgb = mat_vec_mul(
            m_inv,
            [
                (meas.white.x - bx).max(0.0),
                (meas.white.y - by).max(0.0),
                (meas.white.z - bz).max(0.0),
            ],
        );

        let mut gains = [1.0f64; 3];
        for c in 0..3 {
            if native_w_rgb[c] > 1e-6 && raw_d65_rgb[c] > 1e-6 {
                gains[c] = raw_d65_rgb[c] / native_w_rgb[c];
            }
        }
        let max_g = gains[0].max(gains[1]).max(gains[2]).max(1e-6);
        for c in 0..3 {
            gains[c] = (gains[c] / max_g).clamp(0.5, 1.0);
        }

        // Extract per-channel optical output across the measured grayscale ramp
        let mut curves: [Vec<(f64, f64)>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for &(u, ref reading) in &meas.grayscale {
            let dxyz = [
                (reading.x - bx).max(0.0),
                (reading.y - by).max(0.0),
                (reading.z - bz).max(0.0),
            ];
            let rgb_lin = mat_vec_mul(m_inv, dxyz);
            for c in 0..3 {
                let norm_c = if native_w_rgb[c] > 1e-6 {
                    (rgb_lin[c] / native_w_rgb[c]).clamp(0.0, 1.0)
                } else {
                    ((reading.y - lb) / net_lw).clamp(0.0, 1.0)
                };
                curves[c].push((u.clamp(0.0, 1.0), norm_c));
            }
        }

        (gains, curves)
    } else {
        // Fallback to luminance-only grayscale curve if primary matrix is degenerate
        let mut lum_curve = Vec::new();
        for &(u, ref reading) in &meas.grayscale {
            let norm_y = ((reading.y - lb) / net_lw).clamp(0.0, 1.0);
            lum_curve.push((u.clamp(0.0, 1.0), norm_y));
        }
        ([1.0, 1.0, 1.0], [lum_curve.clone(), lum_curve.clone(), lum_curve])
    };

    // Enforce strict monotonicity + endpoint anchoring (0.0 -> 0.0, 1.0 -> 1.0) on each channel curve
    let mut clean_curves: [Vec<(f64, f64)>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for c in 0..3 {
        let mut pts = channel_curves[c].clone();
        pts.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        if pts.first().map(|p| p.0 > 0.001).unwrap_or(true) {
            pts.insert(0, (0.0, 0.0));
        } else {
            pts[0] = (0.0, 0.0);
        }
        if pts.last().map(|p| p.0 < 0.999).unwrap_or(true) {
            pts.push((1.0, 1.0));
        } else {
            let last_idx = pts.len() - 1;
            pts[last_idx] = (1.0, 1.0);
        }
        let mut running_max = 0.0f64;
        for pt in &mut pts {
            running_max = running_max.max(pt.1);
            pt.1 = running_max;
        }
        clean_curves[c] = pts;
    }

    let mut vcgt: [Vec<u16>; 3] = [
        Vec::with_capacity(1024),
        Vec::with_capacity(1024),
        Vec::with_capacity(1024),
    ];

    for i in 0..1024 {
        let v = i as f64 / 1023.0;
        let target_lum = eotf_target_luminance(v, lw, lb, eotf);
        let target_norm = ((target_lum - lb) / net_lw).clamp(0.0, 1.0);

        for c in 0..3 {
            // Desired linear optical output for channel c (scaled by D65 whitepoint gain)
            let desired_y = (target_norm * d65_gains[c]).clamp(0.0, 1.0);
            // Invert the panel's measured response M_c(u) = desired_y -> drive signal u*
            let drive_u = invert_measured_curve(&clean_curves[c], desired_y);
            let u16_val = (drive_u * 65535.0).round() as u16;
            vcgt[c].push(u16_val);
        }
    }

    let mid_idx = 512;
    let midtone_50_vcgt = [
        vcgt[0][mid_idx] as f64 / 65535.0,
        vcgt[1][mid_idx] as f64 / 65535.0,
        vcgt[2][mid_idx] as f64 / 65535.0,
    ];

    (
        vcgt,
        VcgtSummary {
            d65_rgb_gains: d65_gains,
            midtone_50_vcgt,
            used_measured_inversion: true,
        },
    )
}

fn attach_metadata_and_vcgt(
    profile: &mut Profile,
    eotf: EotfConfig,
    vcgt_tables: &[Vec<u16>; 3],
) {
    let mut desc = MLU::new(1);
    desc.set_text(
        &format!("Yutani ASUS PA32UCDM - {}", eotf.display_label()),
        Locale::none(),
    );
    profile.write_tag(TagSignature::ProfileDescriptionTag, Tag::MLU(&desc));

    let mut copy = MLU::new(1);
    copy.set_text(
        "Copyright 2026 Visitant Media LLC - Generated by Yutani",
        Locale::none(),
    );
    profile.write_tag(TagSignature::CopyrightTag, Tag::MLU(&copy));

    let vcgt_r = ToneCurve::new_tabulated(&vcgt_tables[0]);
    let vcgt_g = ToneCurve::new_tabulated(&vcgt_tables[1]);
    let vcgt_b = ToneCurve::new_tabulated(&vcgt_tables[2]);
    let vcgt_refs: [&ToneCurveRef; 3] = [&vcgt_r, &vcgt_g, &vcgt_b];
    profile.write_tag(TagSignature::VcgtTag, Tag::VcgtCurves(vcgt_refs));
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

    // Baseline linear VCGT
    let mut ident = Vec::with_capacity(1024);
    for i in 0..1024 {
        ident.push(((i as f64 / 1023.0) * 65535.0).round() as u16);
    }
    attach_metadata_and_vcgt(&mut profile, eotf, &[ident.clone(), ident.clone(), ident]);

    let filename = eotf.filename_slug();
    profile
        .save_profile_to_file(Path::new(&filename))
        .unwrap();
    println!("Saved {filename}");
}

/// Generate a calibrated ICC profile from real Spyder sensor measurements and chosen EOTF,
/// computing the inverted 3-channel `vcgt` LUTs and saving to both the working directory
/// and `~/.local/share/icc/`.
pub fn generate_profile_from_measurements(
    meas: &CalibrationMeasurements,
    eotf: EotfConfig,
) -> Result<(Vec<PathBuf>, VcgtSummary)> {
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

    // Compute real measured-curve inversion & D65 grey-balance VCGT tables
    let (vcgt_tables, vcgt_summary) = compute_vcgt_tables(meas, eotf);
    attach_metadata_and_vcgt(&mut profile, eotf, &vcgt_tables);

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

    Ok((saved_paths, vcgt_summary))
}

