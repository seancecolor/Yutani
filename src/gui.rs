use std::path::PathBuf;
use std::time::{Duration, Instant};

use eframe::egui;

use crate::driver::spyder::{SpyderSession, XyzReading};
use crate::icc::{
    bt1886_params, bt1886_target_luminance, generate_profile_from_measurements,
    CalibrationMeasurements,
};

/// Time to let the display panel settle on a new color patch before triggering the sensor read.
const PATCH_SETTLE_DURATION: Duration = Duration::from_millis(450);

pub fn run_gui() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_app_id("yutani.calibration")
            .with_inner_size([960.0, 680.0])
            .with_fullscreen(false),
        ..Default::default()
    };

    eframe::run_native(
        "YUTANI // MU-TH-UR 6000 COLORIMETRY SUBSYSTEM",
        options,
        Box::new(|cc| {
            let mut style = egui::Style::default();
            let bg_black = egui::Color32::from_rgb(8, 8, 8);
            let amber = egui::Color32::from_rgb(255, 180, 40);
            let dark_amber = egui::Color32::from_rgb(45, 28, 8);
            let active_amber = egui::Color32::from_rgb(85, 52, 12);

            style.visuals.dark_mode = true;
            style.visuals.window_fill = bg_black;
            style.visuals.panel_fill = bg_black;
            style.visuals.override_text_color = Some(amber);
            style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0_f32, amber);
            style.visuals.widgets.noninteractive.bg_stroke =
                egui::Stroke::new(1.0_f32, egui::Color32::from_rgb(90, 60, 15));
            style.visuals.widgets.inactive.bg_fill = dark_amber;
            style.visuals.widgets.inactive.fg_stroke = egui::Stroke::new(1.0_f32, amber);
            style.visuals.widgets.hovered.bg_fill = active_amber;
            style.visuals.widgets.hovered.fg_stroke = egui::Stroke::new(1.5_f32, amber);
            style.visuals.widgets.active.bg_fill = amber;
            style.visuals.widgets.active.fg_stroke = egui::Stroke::new(1.5_f32, bg_black);

            cc.egui_ctx.set_style(style);

            Box::new(YutaniApp::new())
        }),
    )
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

enum AppPhase {
    /// Main MU-TH-UR 6000 control console.
    Dashboard,
    /// Fullscreen target reticle for physical sensor placement on the monitor.
    PlacementReticle,
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
        saved_paths: Vec<PathBuf>,
    },
}

struct YutaniApp {
    session: Option<SpyderSession>,
    status_log: String,
    last_single_read: Option<XyzReading>,
    phase: AppPhase,
}

impl YutaniApp {
    fn new() -> Self {
        let mut app = Self {
            session: None,
            status_log: String::from("SYSTEM READY. INITIALISING INSTRUMENT LINK..."),
            last_single_read: None,
            phase: AppPhase::Dashboard,
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

        match generate_profile_from_measurements(&measurements) {
            Ok(saved_paths) => {
                self.status_log = format!(
                    "CALIBRATION COMPLETE // SAVED {} ICC PROFILE(S)",
                    saved_paths.len()
                );
                self.phase = AppPhase::Report {
                    measurements,
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
        // Global ESC key aborts fullscreen placement or measurement
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            if !matches!(self.phase, AppPhase::Dashboard) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Fullscreen(false));
                self.status_log = String::from("SEQUENCE ABORTED BY OPERATOR.");
                self.phase = AppPhase::Dashboard;
            }
        }

        match &mut self.phase {
            AppPhase::Dashboard => {
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("YUTANI CORP // MU-TH-UR 6000 COLORIMETRY SUBSYSTEM");
                    ui.label("WEYLAND-YUTANI DISPLAY PROFILER // BUILD BETTER WORLDS");
                    ui.separator();

                    ui.add_space(8.0);
                    ui.label("DISPLAY TARGET : ASUS ProArt PA32UCDM");
                    ui.label("TARGET EOTF    : ITU-R BT.1886 (γ = 2.40 + Black Level Compensation)");
                    ui.label("TARGET WHITE   : CIE D65 (x = 0.3127, y = 0.3290)");
                    ui.label(format!("SENSOR STATUS  : {}", self.status_log));

                    if let Some(s) = &self.session {
                        ui.add_space(6.0);
                        ui.label(format!(
                            "MATRIX ROW Y   : [{:+.6}, {:+.6}, {:+.6}] (setup_off = {:?})",
                            s.matrix[1][0], s.matrix[1][1], s.matrix[1][2], s.setup_off
                        ));
                    }

                    ui.add_space(16.0);
                    ui.horizontal(|ui| {
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
                let mut begin_sequence = start_now;
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
                                if ui.button(" [ ENGAGE 14-PATCH BT.1886 SEQUENCE (SPACE) ] ").clicked() {
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
                saved_paths,
            } => {
                let mut back_to_dash = false;
                egui::CentralPanel::default().show(ctx, |ui| {
                    ui.heading("YUTANI CORP // BT.1886 CALIBRATION REPORT");
                    ui.separator();

                    let lw = measurements.white.y;
                    let lb = measurements.black.y;
                    let contrast = if lb > 1e-5 { lw / lb } else { 0.0 };
                    let (a, b) = bt1886_params(lw, lb);
                    let (wx, wy, _) = measurements.white.to_xyy();
                    let (rx, ry, _) = measurements.red.to_xyy();
                    let (gx, gy, _) = measurements.green.to_xyy();
                    let (bx, by, _) = measurements.blue.to_xyy();

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
                    ui.label(format!(
                        "BT.1886 PARAMS  : a = {:.4}, b = {:.6} (γ = 2.40)",
                        a, b
                    ));
                    ui.label(format!(
                        "PRIMARIES (xy)  : R({:.4}, {:.4})  G({:.4}, {:.4})  B({:.4}, {:.4})",
                        rx, ry, gx, gy, bx, by
                    ));

                    ui.add_space(10.0);
                    ui.separator();
                    ui.label("GRAYSCALE EOTF TRACKING (MEASURED vs. BT.1886 TARGET):");
                    egui::ScrollArea::vertical()
                        .max_height(240.0)
                        .show(ui, |ui| {
                            for (v, reading) in &measurements.grayscale {
                                let target_y = bt1886_target_luminance(*v, lw, lb);
                                let (gx, gy, meas_y) = reading.to_xyy();
                                ui.monospace(format!(
                                    "  {:>3.0}% | Meas: {:>7.3} cd/m² | BT.1886 Target: {:>7.3} cd/m² | xy: ({:.4}, {:.4})",
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

