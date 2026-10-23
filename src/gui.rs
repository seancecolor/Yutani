use std::path::PathBuf;
use std::time::{Duration, Instant};

use eframe::egui;

use crate::driver::spyder::{SpyderSession, XyzReading};
use crate::icc::{
    bt1886_params, eotf_target_luminance, generate_profile_from_measurements,
    CalibrationMeasurements, EotfConfig, VcgtSummary,
};

/// Time to let the display panel settle on a new color patch before triggering the sensor read.
const PATCH_SETTLE_DURATION: Duration = Duration::from_millis(450);

pub fn run_gui() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id("yutani.calibration")
            .with_inner_size([960.0, 680.0])
            .with_fullscreen(false),
        follow_system_theme: false,
        default_theme: eframe::Theme::Dark,
        ..Default::default()
    };

    eframe::run_native(
        "YUTANI // COLORIMETRY SUBSYSTEM",
        options,
        Box::new(|cc| {
            configure_yutani_theme(&cc.egui_ctx);
            Box::new(YutaniApp::new())
        }),
    )
}

fn configure_yutani_theme(ctx: &egui::Context) {
    let mut style = (*ctx.style()).clone();
    let bg_black = egui::Color32::from_rgb(8, 8, 8);
    let panel_dark = egui::Color32::from_rgb(14, 12, 8);
    let amber = egui::Color32::from_rgb(255, 180, 40);
    let dim_amber = egui::Color32::from_rgb(140, 95, 20);
    let dark_amber = egui::Color32::from_rgb(45, 28, 8);
    let hover_amber = egui::Color32::from_rgb(85, 52, 12);

    style.override_font_id = Some(egui::FontId::monospace(14.0));
    style.spacing.item_spacing = egui::vec2(10.0, 8.0);
    style.spacing.button_padding = egui::vec2(12.0, 8.0);

    let mut visuals = egui::Visuals::dark();
    visuals.window_fill = bg_black;
    visuals.panel_fill = panel_dark;
    visuals.faint_bg_color = bg_black;
    visuals.extreme_bg_color = bg_black;
    visuals.override_text_color = Some(amber);
    visuals.window_rounding = egui::Rounding::ZERO;
    visuals.menu_rounding = egui::Rounding::ZERO;

    visuals.widgets.noninteractive.bg_fill = bg_black;
    visuals.widgets.noninteractive.weak_bg_fill = bg_black;
    visuals.widgets.noninteractive.bg_stroke = egui::Stroke::new(1.0_f32, dim_amber);
    visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0_f32, amber);
    visuals.widgets.noninteractive.rounding = egui::Rounding::ZERO;

    visuals.widgets.inactive.bg_fill = dark_amber;
    visuals.widgets.inactive.weak_bg_fill = dark_amber;
    visuals.widgets.inactive.bg_stroke = egui::Stroke::new(1.0_f32, amber);
    visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0_f32, amber);
    visuals.widgets.inactive.rounding = egui::Rounding::ZERO;

    visuals.widgets.hovered.bg_fill = hover_amber;
    visuals.widgets.hovered.weak_bg_fill = hover_amber;
    visuals.widgets.hovered.bg_stroke = egui::Stroke::new(1.5_f32, amber);
    visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.5_f32, amber);
    visuals.widgets.hovered.rounding = egui::Rounding::ZERO;

    visuals.widgets.active.bg_fill = amber;
    visuals.widgets.active.weak_bg_fill = amber;
    visuals.widgets.active.bg_stroke = egui::Stroke::new(1.5_f32, amber);
    visuals.widgets.active.fg_stroke = egui::Stroke::new(1.5_f32, bg_black);
    visuals.widgets.active.rounding = egui::Rounding::ZERO;

    style.visuals = visuals;
    ctx.set_style(style);
}

#[derive(Clone, Copy, Debug)]
struct PatchSpec {
    label: &'static str,
    color: egui::Color32,
    kind: PatchKind,
}

#[derive(Clone, Copy, Debug)]
enum PatchKind {
    Black,
    White,
    Red,
    Green,
    Blue,
    Gray(f64),
}

fn build_calibration_patches() -> Vec<PatchSpec> {
    let mut patches = vec![
        PatchSpec {
            label: "01 // BLACK LEVEL (0%)",
            color: egui::Color32::from_rgb(0, 0, 0),
            kind: PatchKind::Black,
        },
        PatchSpec {
            label: "02 // PEAK WHITE (100%)",
            color: egui::Color32::from_rgb(255, 255, 255),
            kind: PatchKind::White,
        },
        PatchSpec {
            label: "03 // RED PRIMARY",
            color: egui::Color32::from_rgb(255, 0, 0),
            kind: PatchKind::Red,
        },
        PatchSpec {
            label: "04 // GREEN PRIMARY",
            color: egui::Color32::from_rgb(0, 255, 0),
            kind: PatchKind::Green,
        },
        PatchSpec {
            label: "05 // BLUE PRIMARY",
            color: egui::Color32::from_rgb(0, 0, 255),
            kind: PatchKind::Blue,
        },
    ];

    let gray_steps: [f64; 9] = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
    for (idx, &v) in gray_steps.iter().enumerate() {
        let level = (v * 255.0).round() as u8;
        let label: &'static str = match idx {
            0 => "06 // GRAY 10%",
            1 => "07 // GRAY 20%",
            2 => "08 // GRAY 30%",
            3 => "09 // GRAY 40%",
            4 => "10 // GRAY 50%",
            5 => "11 // GRAY 60%",
            6 => "12 // GRAY 70%",
            7 => "13 // GRAY 80%",
            _ => "14 // GRAY 90%",
        };
        patches.push(PatchSpec {
            label,
            color: egui::Color32::from_rgb(level, level, level),
            kind: PatchKind::Gray(v),
        });
    }

    patches
}

/// Interval between consecutive sensor reads in Live OSD Alignment mode.
const OSD_POLL_INTERVAL: Duration = Duration::from_millis(150);

/// Compute D65-normalized linear RGB balance `(r_pct, g_pct, b_pct)`, Correlated Color
/// Temperature `cct_kelvin`, and CIE 1976 `delta_e_ab` relative to CIE D65 whitepoint.
fn compute_d65_alignment_metrics(reading: &XyzReading) -> ([f64; 3], f64, f64) {
    let (cx, cy, lum) = reading.to_xyy();
    if lum <= 1e-4 {
        return ([100.0, 100.0, 100.0], 6504.0, 99.9);
    }

    // Standard CIE 1931 XYZ -> Linear BT.709 / sRGB (D65 white => R = G = B = Y)
    let x = reading.x;
    let y = reading.y;
    let z = reading.z;
    let r_lin = (3.2404542 * x - 1.5371385 * y - 0.4985314 * z).max(0.0);
    let g_lin = (-0.9692660 * x + 1.8760108 * y + 0.0415560 * z).max(0.0);
    let b_lin = (0.0556434 * x - 0.2040259 * y + 1.0572252 * z).max(0.0);

    let avg = ((r_lin + g_lin + b_lin) / 3.0).max(1e-6);
    let rgb_pct = [
        (r_lin / avg) * 100.0,
        (g_lin / avg) * 100.0,
        (b_lin / avg) * 100.0,
    ];

    // McCamy's cubic approximation for Correlated Color Temperature (CCT in Kelvin)
    let denom = (0.1858 - cy).abs().max(1e-6) * if 0.1858 - cy >= 0.0 { 1.0 } else { -1.0 };
    let n = (cx - 0.3320) / denom;
    let cct = (449.0 * n.powi(3) + 3525.0 * n.powi(2) + 6823.3 * n + 5520.33).clamp(1500.0, 25000.0);

    // Chromaticity CIE Lab a*, b* delta against D65 (Xn=0.95047, Yn=1.0, Zn=1.08883) at equal luminance
    let xn = y * (0.3127 / 0.3290);
    let zn = y * ((1.0 - 0.3127 - 0.3290) / 0.3290);
    let f = |t: f64| -> f64 {
        let delta: f64 = 6.0 / 29.0;
        if t > delta.powi(3) {
            t.cbrt()
        } else {
            t / (3.0 * delta.powi(2)) + 4.0 / 29.0
        }
    };
    let fx = f(x / xn.max(1e-6));
    let fy = f(1.0);
    let fz = f(z / zn.max(1e-6));
    let a_star = 500.0 * (fx - fy);
    let b_star = 200.0 * (fy - fz);
    let delta_e = (a_star.powi(2) + b_star.powi(2)).sqrt();

    (rgb_pct, cct, delta_e)
}

/// Draw a retro MU-TH-UR 6000 horizontal center-zero deviation gauge bar.
fn draw_deviation_bar(
    ui: &mut egui::Ui,
    label: &str,
    pct_diff: f64,
    max_span_pct: f64,
    bar_color: egui::Color32,
    hint: &str,
) {
    let amber = egui::Color32::from_rgb(255, 180, 40);
    let dim_amber = egui::Color32::from_rgb(90, 60, 15);
    let bg_dark = egui::Color32::from_rgb(16, 14, 10);

    ui.horizontal(|ui| {
        ui.monospace(format!("{label:<14}"));
        let (rect, _) = ui.allocate_exact_size(egui::vec2(280.0, 18.0), egui::Sense::hover());
        let painter = ui.painter();

        painter.rect_filled(rect, 0.0, bg_dark);
        painter.rect_stroke(rect, 0.0, egui::Stroke::new(1.0_f32, dim_amber));

        let center_x = rect.center().x;
        let norm = (pct_diff / max_span_pct).clamp(-1.0, 1.0) as f32;
        let fill_x = center_x + norm * (rect.width() * 0.5);

        let bar_rect = if fill_x >= center_x {
            egui::Rect::from_min_max(
                egui::pos2(center_x, rect.top() + 2.0),
                egui::pos2(fill_x, rect.bottom() - 2.0),
            )
        } else {
            egui::Rect::from_min_max(
                egui::pos2(fill_x, rect.top() + 2.0),
                egui::pos2(center_x, rect.bottom() - 2.0),
            )
        };
        painter.rect_filled(bar_rect, 0.0, bar_color);

        // Center target tick line
        painter.line_segment(
            [
                egui::pos2(center_x, rect.top()),
                egui::pos2(center_x, rect.bottom()),
            ],
            egui::Stroke::new(2.0_f32, amber),
        );

        ui.monospace(format!(" {:+6.1}%  {}", pct_diff, hint));
    });
}

enum AppPhase {
    /// Main MU-TH-UR 6000 control console.
    Dashboard,
    /// Fullscreen target reticle for physical sensor placement on the monitor.
    PlacementReticle,
    /// Live continuous OSD Whitepoint (D65) & Luminance (Nits) alignment meter.
    OsdAlignment {
        last_poll: Instant,
        paused: bool,
        reading: Option<XyzReading>,
        read_count: usize,
    },
    /// Active fullscreen patch display and hardware measurement sequence.
    Measuring {
        patches: Vec<PatchSpec>,
        current_index: usize,
        patch_shown_at: Instant,
        awaiting_read: bool,
        readings: Vec<(PatchSpec, XyzReading)>,
    },
    /// Completed calibration telemetry report.
    Report {
        measurements: CalibrationMeasurements,
        vcgt_summary: VcgtSummary,
        saved_paths: Vec<PathBuf>,
    },
}

struct YutaniApp {
    session: Option<SpyderSession>,
    status_log: String,
    last_single_read: Option<XyzReading>,
    eotf: EotfConfig,
    target_nits: f64,
    phase: AppPhase,
    theme_applied: bool,
}

impl YutaniApp {
    fn new() -> Self {
        let mut app = Self {
            session: None,
            status_log: String::from("SYSTEM READY. INITIALISING INSTRUMENT LINK..."),
            last_single_read: None,
            eotf: EotfConfig::default(),
            target_nits: 100.0,
            phase: AppPhase::Dashboard,
            theme_applied: false,
        };
        app.connect_sensor();
        app
    }

    fn connect_sensor(&mut self) {
        match SpyderSession::open() {
            Ok(s) => {
                self.status_log = format!(
                    "ONLINE: {} [{}] // FW+SN: {} // INT: {} ms",
                    s.model_name,
                    s.cmd_set.name(),
                    s.firmware_serial,
                    s.integration_ms
                );
                self.session = Some(s);
            }
            Err(e) => {
                self.status_log = format!("OFFLINE: {e:#}");
                self.session = None;
            }
        }
    }

    fn finalize_measurements(
        &mut self,
        ctx: &egui::Context,
        readings: Vec<(PatchSpec, XyzReading)>,
    ) {
        ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));

        let mut black = XyzReading::default();
        let mut white = XyzReading::default();
        let mut red = XyzReading::default();
        let mut green = XyzReading::default();
        let mut blue = XyzReading::default();
        let mut grayscale = Vec::new();

        for (spec, reading) in readings {
            match spec.kind {
                PatchKind::Black => {
                    black = reading;
                    grayscale.push((0.0, reading));
                }
                PatchKind::White => {
                    white = reading;
                    grayscale.push((1.0, reading));
                }
                PatchKind::Red => red = reading,
                PatchKind::Green => green = reading,
                PatchKind::Blue => blue = reading,
                PatchKind::Gray(v) => grayscale.push((v, reading)),
            }
        }

        grayscale.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let measurements = CalibrationMeasurements {
            black,
            white,
            red,
            green,
            blue,
            grayscale,
        };

        match generate_profile_from_measurements(&measurements, self.eotf) {
            Ok((saved_paths, vcgt_summary)) => {
                self.status_log = format!(
                    "CALIBRATION COMPLETE // SAVED {} ICC PROFILE(S) WITH 3x1024 VCGT",
                    saved_paths.len()
                );
                self.phase = AppPhase::Report {
                    measurements,
                    vcgt_summary,
                    saved_paths,
                };
            }
            Err(e) => {
                self.status_log = format!("ICC GENERATION ERROR: {e:#}");
                self.phase = AppPhase::Dashboard;
            }
        }
    }
}

impl eframe::App for YutaniApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        if !self.theme_applied {
            configure_yutani_theme(ctx);
            self.theme_applied = true;
        }

        // Global ESC key aborts fullscreen placement or measurement
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if !matches!(self.phase, AppPhase::Dashboard) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                self.status_log = String::from("SEQUENCE ABORTED BY OPERATOR.");
                self.phase = AppPhase::Dashboard;
            }
        }

        let current_eotf = self.eotf;

        match &mut self.phase {
            AppPhase::Dashboard => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("YUTANI CORP // WAYLAND COLORIMETRY SUBSYSTEM");
                    ui.label("YUTANI DISPLAY PROFILER // BUILD BETTER COLORS");
                    ui.separator();

                    ui.add_space(8.0);
                    ui.label("DISPLAY TARGET : ASUS ProArt PA32UCDM");
                    ui.label(format!("TARGET EOTF    : {}", self.eotf.display_label()));
                    ui.label(format!("OUTPUT PROFILE : {}", self.eotf.filename_slug()));
                    ui.label("TARGET WHITE   : CIE D65 (x = 0.3127, y = 0.3290)");
                    ui.label(format!("SENSOR STATUS  : {}", self.status_log));

                    if let Some(s) = &self.session {
                        ui.add_space(6.0);
                        ui.label(format!(
                            "MATRIX ROW Y   : [{:+.6}, {:+.6}, {:+.6}] (setup_off = {:?})",
                            s.matrix[1][0], s.matrix[1][1], s.matrix[1][2], s.setup_off
                        ));
                    }

                    ui.add_space(12.0);
                    ui.separator();
                    ui.label("SELECT TARGET GAMMA / EOTF CURVE:");
                    ui.horizontal_wrapped(|ui| {
                        let is_bt1886_24 = matches!(self.eotf, EotfConfig::Bt1886 { gamma } if (gamma - 2.4).abs() < 0.005);
                        if ui.selectable_label(is_bt1886_24, " [ BT.1886 (γ 2.40) ] ").clicked() {
                            self.eotf = EotfConfig::Bt1886 { gamma: 2.4 };
                        }

                        let is_bt1886_22 = matches!(self.eotf, EotfConfig::Bt1886 { gamma } if (gamma - 2.2).abs() < 0.005);
                        if ui.selectable_label(is_bt1886_22, " [ BT.1886 (γ 2.20) ] ").clicked() {
                            self.eotf = EotfConfig::Bt1886 { gamma: 2.2 };
                        }

                        let is_pure_22 = matches!(self.eotf, EotfConfig::PurePower { gamma } if (gamma - 2.2).abs() < 0.005);
                        if ui.selectable_label(is_pure_22, " [ PURE GAMMA 2.20 ] ").clicked() {
                            self.eotf = EotfConfig::PurePower { gamma: 2.2 };
                        }

                        let is_pure_24 = matches!(self.eotf, EotfConfig::PurePower { gamma } if (gamma - 2.4).abs() < 0.005);
                        if ui.selectable_label(is_pure_24, " [ PURE GAMMA 2.40 ] ").clicked() {
                            self.eotf = EotfConfig::PurePower { gamma: 2.4 };
                        }

                        let is_pure_26 = matches!(self.eotf, EotfConfig::PurePower { gamma } if (gamma - 2.6).abs() < 0.005);
                        if ui.selectable_label(is_pure_26, " [ DCI GAMMA 2.60 ] ").clicked() {
                            self.eotf = EotfConfig::PurePower { gamma: 2.6 };
                        }

                        let is_srgb = matches!(self.eotf, EotfConfig::Srgb);
                        if ui.selectable_label(is_srgb, " [ sRGB PIECEWISE ] ").clicked() {
                            self.eotf = EotfConfig::Srgb;
                        }
                    });

                    // Fine-grained custom gamma slider & black-compensation mode toggle
                    if !matches!(self.eotf, EotfConfig::Srgb) {
                        let mut g = self.eotf.gamma_value();
                        let mut use_bt1886 = matches!(self.eotf, EotfConfig::Bt1886 { .. });
                        ui.horizontal(|ui| {
                            ui.label("CUSTOM GAMMA (γ):");
                            let slider_changed = ui
                                .add(
                                    egui::Slider::new(&mut g, 1.60..=2.80)
                                        .step_by(0.05)
                                        .fixed_decimals(2),
                                )
                                .changed();
                            let mode_changed = ui
                                .checkbox(
                                    &mut use_bt1886,
                                    "BT.1886 Black-Level Lift Compensation",
                                )
                                .changed();
                            if slider_changed || mode_changed {
                                self.eotf = if use_bt1886 {
                                    EotfConfig::Bt1886 { gamma: g }
                                } else {
                                    EotfConfig::PurePower { gamma: g }
                                };
                            }
                        });
                    }

                    ui.add_space(8.0);
                    ui.horizontal(|ui| {
                        ui.label("TARGET PEAK LUMINANCE (OSD):");
                        for &preset in &[80.0, 100.0, 120.0, 160.0, 200.0] {
                            let selected = (self.target_nits - preset).abs() < 0.5;
                            if ui
                                .selectable_label(selected, format!(" {:.0} cd/m² ", preset))
                                .clicked()
                            {
                                self.target_nits = preset;
                            }
                        }
                        ui.add(
                            egui::Slider::new(&mut self.target_nits, 40.0..=300.0)
                                .step_by(5.0)
                                .suffix(" cd/m²")
                                .fixed_decimals(0),
                        );
                    });

                    ui.add_space(12.0);
                    ui.separator();
                    ui.horizontal_wrapped(|ui| {
                        if ui.button(" [ RE-SCAN USB INSTRUMENT ] ").clicked() {
                            self.connect_sensor();
                        }

                        if self.session.is_some() {
                            if ui.button(" [ SINGLE READ (TEST SENSOR) ] ").clicked() {
                                if let Some(s) = &self.session {
                                    match s.measure() {
                                        Ok(r) => {
                                            let (x, y, lum) = r.to_xyy();
                                            self.status_log = format!(
                                                "READ OK: {:.3} cd/m² (x={:.4}, y={:.4}) raw={:?}",
                                                lum, x, y, r.raw_counts
                                            );
                                            self.last_single_read = Some(r);
                                        }
                                        Err(e) => {
                                            self.status_log = format!("READ FAILED: {e:#}");
                                        }
                                    }
                                 }
                            }

                            if ui.button(" [ LIVE OSD WHITE & NITS METER ] ").clicked() {
                                ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
                                self.phase = AppPhase::OsdAlignment {
                                    last_poll: Instant::now() - OSD_POLL_INTERVAL,
                                    paused: false,
                                    reading: self.last_single_read,
                                    read_count: 0,
                                };
                            }

                            if ui.button(" [ INITIALIZE CALIBRATION SEQUENCE ] ").clicked() {
                                ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(true));
                                self.phase = AppPhase::PlacementReticle;
                            }
                        }
                    });

                    if let Some(r) = self.last_single_read {
                        let (cx, cy, lum) = r.to_xyy();
                        ui.add_space(14.0);
                        ui.separator();
                        ui.label("TELEMETRY // LAST SPOT MEASUREMENT:");
                        ui.label(format!(
                            "  Luminance (Y) : {:.4} cd/m² (nits)",
                            lum
                        ));
                        ui.label(format!(
                            "  CIE 1931 xy   : x = {:.4}, y = {:.4}",
                            cx, cy
                        ));
                        ui.label(format!(
                            "  CIE 1931 XYZ  : X = {:.4}, Y = {:.4}, Z = {:.4}",
                            r.x, r.y, r.z
                        ));
                        ui.label(format!(
                            "  Raw Counts    : R={} G={} B={} (Net: {:.1}, {:.1}, {:.1})",
                            r.raw_counts[0],
                            r.raw_counts[1],
                            r.raw_counts[2],
                            r.net_counts[0],
                            r.net_counts[1],
                            r.net_counts[2]
                        ));
                    }
                });
            }

            AppPhase::PlacementReticle => {
                let start_now = ctx.input(|i| i.key_pressed(egui::Key::Space));
                let open_osd = ctx.input(|i| i.key_pressed(egui::Key::O));
                let mut begin_sequence = start_now;
                let mut launch_osd = open_osd;
                let mut cancel = false;

                egui::CentralPanel::default()
                    .frame(egui::Frame::default().fill(egui::Color32::from_rgb(8, 8, 8)))
                    .show(ctx, |ui| {
                        let rect = ui.max_rect();
                        let center = rect.center();
                        let painter = ui.painter();
                        let amber = egui::Color32::from_rgb(255, 180, 40);

                        // Draw target reticle in exact screen center for hanging the SpyderX2
                        let box_size = egui::vec2(260.0, 260.0);
                        let target_rect = egui::Rect::from_center_size(center, box_size);
                        painter.rect_stroke(
                            target_rect,
                            0.0,
                            egui::Stroke::new(2.0_f32, amber),
                        );
                        painter.circle_stroke(center, 70.0, egui::Stroke::new(1.5_f32, amber));
                        painter.line_segment(
                            [
                                egui::pos2(center.x - 100.0, center.y),
                                egui::pos2(center.x + 100.0, center.y),
                            ],
                            egui::Stroke::new(1.0_f32, amber),
                        );
                        painter.line_segment(
                            [
                                egui::pos2(center.x, center.y - 100.0),
                                egui::pos2(center.x, center.y + 100.0),
                            ],
                            egui::Stroke::new(1.0_f32, amber),
                        );

                        ui.vertical_centered(|ui| {
                            ui.add_space(32.0);
                            ui.heading("ALIGN SPYDER SENSOR INSIDE CENTER RETICLE");
                            ui.label("1. Remove sensor lens cap and place flush against the Asus PA32UCDM panel.");
                            ui.label("2. Ensure KWin ICC profile is set to 'None' during raw panel characterization.");
                            ui.add_space(16.0);
                            ui.horizontal(|ui| {
                                if ui.button(" [ LIVE OSD WHITE & NITS METER (O) ] ").clicked() {
                                    launch_osd = true;
                                }
                                if ui.button(" [ ENGAGE 14-PATCH SEQUENCE (SPACE) ] ").clicked() {
                                    begin_sequence = true;
                                }
                                if ui.button(" [ ABORT (ESC) ] ").clicked() {
                                    cancel = true;
                                }
                            });
                        });
                    });

                if cancel {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                    self.phase = AppPhase::Dashboard;
                } else if launch_osd {
                    self.phase = AppPhase::OsdAlignment {
                        last_poll: Instant::now() - OSD_POLL_INTERVAL,
                        paused: false,
                        reading: None,
                        read_count: 0,
                    };
                    ctx.request_repaint();
                } else if begin_sequence {
                    self.phase = AppPhase::Measuring {
                        patches: build_calibration_patches(),
                        current_index: 0,
                        patch_shown_at: Instant::now(),
                        awaiting_read: false,
                        readings: Vec::with_capacity(14),
                    };
                    ctx.request_repaint();
                }
            }

            AppPhase::OsdAlignment {
                last_poll,
                paused,
                reading,
                read_count,
            } => {
                let start_cal = ctx.input(|i| i.key_pressed(egui::Key::Space));
                let mut proceed_to_cal = start_cal;
                let mut exit_to_dash = false;
                let mut trigger_manual_read = false;

                egui::CentralPanel::default()
                    .frame(egui::Frame::default().fill(egui::Color32::from_rgb(8, 8, 8)))
                    .show(ctx, |ui| {
                        let rect = ui.max_rect();
                        let center = rect.center();
                        let painter = ui.painter();
                        let amber = egui::Color32::from_rgb(255, 180, 40);

                        // Pure 100% White (255, 255, 255) patch in the exact center for the Spyder sensor
                        let patch_size = egui::vec2(300.0, 300.0);
                        let patch_rect = egui::Rect::from_center_size(center, patch_size);
                        painter.rect_filled(patch_rect, 0.0, egui::Color32::WHITE);
                        painter.rect_stroke(
                            patch_rect.expand(4.0),
                            0.0,
                            egui::Stroke::new(2.0_f32, amber),
                        );

                        // Top telemetry HUD (outside the center optical patch)
                        ui.vertical(|ui| {
                            ui.heading(
                                "MU-TH-UR 6000 // LIVE OSD WHITE-POINT (D65) & LUMINANCE ALIGNMENT",
                            );
                            ui.label(
                                "Adjust Asus PA32UCDM OSD Brightness & RGB Gain while sensor reads center white patch.",
                            );
                            ui.separator();

                            ui.horizontal(|ui| {
                                ui.label(format!(
                                    "TARGET LUMINANCE: {:.0} cd/m²  |  TARGET WHITE: CIE D65 (x=0.3127, y=0.3290, 6504K)  |  SAMPLES: {}",
                                    self.target_nits, *read_count
                                ));
                            });

                            if let Some(r) = *reading {
                                let (cx, cy, lum) = r.to_xyy();
                                let (rgb_pct, cct, delta_e) = compute_d65_alignment_metrics(&r);
                                let lum_diff_nits = lum - self.target_nits;
                                let lum_diff_pct =
                                    (lum_diff_nits / self.target_nits.max(1.0)) * 100.0;

                                let lum_hint = if lum_diff_nits.abs() <= 2.0 {
                                    "[ LUMINANCE LOCKED ]"
                                } else if lum_diff_nits > 0.0 {
                                    "LOWER OSD BRIGHTNESS"
                                } else {
                                    "RAISE OSD BRIGHTNESS"
                                };

                                draw_deviation_bar(
                                    ui,
                                    &format!("Y ({:.1} nit)", lum),
                                    lum_diff_pct,
                                    30.0,
                                    egui::Color32::from_rgb(255, 190, 60),
                                    lum_hint,
                                );

                                let r_diff = rgb_pct[0] - 100.0;
                                let g_diff = rgb_pct[1] - 100.0;
                                let b_diff = rgb_pct[2] - 100.0;

                                draw_deviation_bar(
                                    ui,
                                    &format!("RED ({:.1}%)", rgb_pct[0]),
                                    r_diff,
                                    15.0,
                                    egui::Color32::from_rgb(235, 65, 55),
                                    if r_diff.abs() <= 1.0 {
                                        "[ R LOCKED ]"
                                    } else if r_diff > 0.0 {
                                        "LOWER OSD RED GAIN"
                                    } else {
                                        "RAISE OSD RED GAIN"
                                    },
                                );
                                draw_deviation_bar(
                                    ui,
                                    &format!("GRN ({:.1}%)", rgb_pct[1]),
                                    g_diff,
                                    15.0,
                                    egui::Color32::from_rgb(65, 220, 85),
                                    if g_diff.abs() <= 1.0 {
                                        "[ G LOCKED ]"
                                    } else if g_diff > 0.0 {
                                        "LOWER OSD GREEN GAIN"
                                    } else {
                                        "RAISE OSD GREEN GAIN"
                                    },
                                );
                                draw_deviation_bar(
                                    ui,
                                    &format!("BLU ({:.1}%)", rgb_pct[2]),
                                    b_diff,
                                    15.0,
                                    egui::Color32::from_rgb(70, 145, 255),
                                    if b_diff.abs() <= 1.0 {
                                        "[ B LOCKED ]"
                                    } else if b_diff > 0.0 {
                                        "LOWER OSD BLUE GAIN"
                                    } else {
                                        "RAISE OSD BLUE GAIN"
                                    },
                                );

                                let lock_status =
                                    if delta_e < 1.0 && lum_diff_nits.abs() <= 2.0 {
                                        "STATUS: [ D65 & LUMINANCE LOCKED // READY FOR 14-PATCH SWEEP ]"
                                    } else if delta_e < 2.0 {
                                        "STATUS: [ NEAR D65 SPEC (ΔE < 2.0) // FINE-TUNE OSD ]"
                                    } else {
                                        "STATUS: [ ADJUSTING OSD // ALIGN BARS TO CENTER TICK ]"
                                    };

                                ui.monospace(format!(
                                    "CIE xy = ({:.4}, {:.4})  |  CCT = {:.0} K  |  ΔE(D65) = {:.2}  |  {}",
                                    cx, cy, cct, delta_e, lock_status
                                ));
                            } else {
                                ui.label("ACQUIRING INITIAL SENSOR READING FROM CENTER WHITE PATCH...");
                            }

                            ui.add_space(6.0);
                            ui.horizontal(|ui| {
                                let pause_label = if *paused {
                                    " [ RESUME LIVE POLLING ] "
                                } else {
                                    " [ PAUSE LIVE POLLING ] "
                                };
                                if ui.button(pause_label).clicked() {
                                    *paused = !*paused;
                                }
                                if *paused && ui.button(" [ SINGLE READ ] ").clicked() {
                                    trigger_manual_read = true;
                                }
                                if ui
                                    .button(" [ START 14-PATCH CALIBRATION (SPACE) ] ")
                                    .clicked()
                                {
                                    proceed_to_cal = true;
                                }
                                if ui.button(" [ EXIT TO CONSOLE (ESC) ] ").clicked() {
                                    exit_to_dash = true;
                                }
                            });
                        });
                    });

                if exit_to_dash {
                    ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                    self.phase = AppPhase::Dashboard;
                } else if proceed_to_cal {
                    self.phase = AppPhase::Measuring {
                        patches: build_calibration_patches(),
                        current_index: 0,
                        patch_shown_at: Instant::now(),
                        awaiting_read: false,
                        readings: Vec::with_capacity(14),
                    };
                    ctx.request_repaint();
                } else if (!*paused && last_poll.elapsed() >= OSD_POLL_INTERVAL)
                    || trigger_manual_read
                {
                    if let Some(s) = &self.session {
                        if let Ok(r) = s.measure() {
                            *reading = Some(r);
                            self.last_single_read = Some(r);
                            *read_count += 1;
                        }
                    }
                    *last_poll = Instant::now();
                    if !*paused {
                        ctx.request_repaint_after(OSD_POLL_INTERVAL);
                    }
                } else if !*paused {
                    ctx.request_repaint_after(OSD_POLL_INTERVAL - last_poll.elapsed());
                }
            }

            AppPhase::Measuring {
                patches,
                current_index,
                patch_shown_at,
                awaiting_read,
                readings,
            } => {
                let spec = patches[*current_index];

                // Fill entire screen with the active calibration patch color
                egui::CentralPanel::default()
                    .frame(egui::Frame::default().fill(spec.color))
                    .show(ctx, |ui| {
                        // Small top-left corner status outside the center sensor zone
                        egui::Frame::default()
                            .fill(egui::Color32::from_rgba_unmultiplied(0, 0, 0, 200))
                            .inner_margin(8.0)
                            .show(ui, |ui| {
                                ui.label(format!(
                                    "YUTANI // PATCH {}/{} : {}  [ESC TO ABORT]",
                                    *current_index + 1,
                                    patches.len(),
                                    spec.label
                                ));
                            });
                    });

                let elapsed = patch_shown_at.elapsed();
                if elapsed < PATCH_SETTLE_DURATION {
                    // Keep repainting until the panel settle window completes
                    ctx.request_repaint_after(PATCH_SETTLE_DURATION - elapsed);
                } else if !*awaiting_read {
                    // First frame after settle duration: ensure the patch frame is rendered, then trigger read on next frame
                    *awaiting_read = true;
                    ctx.request_repaint();
                } else {
                    // Perform synchronous hardware read for this settled patch
                    let read_res = if let Some(s) = &self.session {
                        s.measure()
                    } else {
                        Err(anyhow::anyhow!("Spyder session disconnected"))
                    };

                    match read_res {
                        Ok(reading) => {
                            readings.push((spec, reading));
                            *current_index += 1;
                            if *current_index >= patches.len() {
                                let completed = std::mem::take(readings);
                                self.finalize_measurements(ctx, completed);
                            } else {
                                *patch_shown_at = Instant::now();
                                *awaiting_read = false;
                                ctx.request_repaint();
                            }
                        }
                        Err(e) => {
                            ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                            self.status_log = format!("MEASUREMENT ERROR ON {}: {e:#}", spec.label);
                            self.phase = AppPhase::Dashboard;
                        }
                    }
                }
            }

            AppPhase::Report {
                measurements,
                vcgt_summary,
                saved_paths,
            } => {
                let mut back_to_dash = false;
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("YUTANI CORP // CALIBRATION TELEMETRY REPORT");
                    ui.separator();

                    let lw = measurements.white.y;
                    let lb = measurements.black.y;
                    let contrast = if lb > 1e-5 { lw / lb } else { 0.0 };
                    let (a, b) = bt1886_params(lw, lb, current_eotf.gamma_value());
                    let (wx, wy, _) = measurements.white.to_xyy();
                    let (rx, ry, _) = measurements.red.to_xyy();
                    let (gx, gy, _) = measurements.green.to_xyy();
                    let (bx, by, _) = measurements.blue.to_xyy();

                    ui.label(format!("TARGET EOTF     : {}", current_eotf.display_label()));
                    ui.label(format!(
                        "PEAK WHITE (Lw) : {:.3} cd/m²  (CIE xy = {:.4}, {:.4})",
                        lw, wx, wy
                    ));
                    ui.label(format!("BLACK LEVEL (Lb): {:.4} cd/m²", lb));
                    if contrast > 0.0 {
                        ui.label(format!("CONTRAST RATIO  : {:.0} : 1", contrast));
                    } else {
                        ui.label("CONTRAST RATIO  : ∞ : 1 (OLED Zero-Black)");
                    }
                    if let EotfConfig::Bt1886 { gamma } = current_eotf {
                        ui.label(format!(
                            "BT.1886 PARAMS  : a = {:.4}, b = {:.6} (γ = {:.2})",
                            a, b, gamma
                        ));
                    }
                    ui.label(format!(
                        "PRIMARIES (xy)  : R({:.4}, {:.4})  G({:.4}, {:.4})  B({:.4}, {:.4})",
                        rx, ry, gx, gy, bx, by
                    ));
                    ui.label(format!(
                        "D65 VCGT GAINS  : R = {:.1}% | G = {:.1}% | B = {:.1}% (Inversion: {})",
                        vcgt_summary.d65_rgb_gains[0] * 100.0,
                        vcgt_summary.d65_rgb_gains[1] * 100.0,
                        vcgt_summary.d65_rgb_gains[2] * 100.0,
                        if vcgt_summary.used_measured_inversion {
                            "ACTIVE"
                        } else {
                            "BASELINE FALLBACK"
                        }
                    ));
                    ui.label(format!(
                        "VCGT @ 50% IN   : R_out = {:.3} | G_out = {:.3} | B_out = {:.3}",
                        vcgt_summary.midtone_50_vcgt[0],
                        vcgt_summary.midtone_50_vcgt[1],
                        vcgt_summary.midtone_50_vcgt[2]
                    ));

                    ui.add_space(10.0);
                    ui.separator();
                    ui.label("GRAYSCALE EOTF TRACKING (MEASURED vs. TARGET):");
                    egui::ScrollArea::vertical()
                        .max_height(240.0)
                        .show(ui, |ui| {
                            for (v, reading) in &measurements.grayscale {
                                let target_y = eotf_target_luminance(*v, lw, lb, current_eotf);
                                let (gx, gy, meas_y) = reading.to_xyy();
                                ui.monospace(format!(
                                    "  {:>3.0}% | Meas: {:>7.3} cd/m² | Target: {:>7.3} cd/m² | xy: ({:.4}, {:.4})",
                                    v * 100.0,
                                    meas_y,
                                    target_y,
                                    gx,
                                    gy
                                ));
                            }
                        });

                    ui.add_space(10.0);
                    ui.separator();
                    ui.label("GENERATED ICC PROFILES:");
                    for p in saved_paths.iter() {
                        ui.monospace(format!("  -> {}", p.display()));
                    }
                    ui.label(
                        "KWin Wayland: System Settings -> Display & Monitor -> Display Configuration -> Color Profile -> Assign ICC Profile.",
                    );

                    ui.add_space(14.0);
                    if ui.button(" [ RETURN TO MU-TH-UR CONSOLE ] ").clicked() {
                        back_to_dash = true;
                    }
                });

                if back_to_dash {
                    self.phase = AppPhase::Dashboard;
                }
            }
        }
    }
}

