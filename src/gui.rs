use eframe::egui;

pub fn run_gui() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        wayland_app_id: Some("yutani.calibration".to_string()),
        fullscreen: false,
        ..Default::default()
    };
    
    eframe::run_native(
        "YUTANI - CALIBRATION PROTOCOL",
        options,
        Box::new(|cc| {
            let mut style = egui::Style::default();
            style.visuals.window_fill = egui::Color32::from_rgb(10, 10, 10);
            style.visuals.panel_fill = egui::Color32::from_rgb(10, 10, 10);
            style.visuals.widgets.noninteractive.fg_stroke = egui::Stroke::new(1.0, egui::Color32::from_rgb(255, 180, 50)); // Amber
            style.visuals.widgets.inactive.bg_fill = egui::Color32::from_rgb(50, 30, 10);
            cc.egui_ctx.set_style(style);
            
            Ok(Box::new(YutaniApp::default()))
        }),
    )
}

#[derive(Default)]
struct YutaniApp {
    calibrating: bool,
}

impl eframe::App for YutaniApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading("YUTANI CORP - COLORIMETRY SUBSYSTEM");
            ui.separator();
            ui.label("Display Target: Asus PA32UCDM");
            ui.label("Target Curve: BT.1886");
            ui.label("Sensor: Datacolor SpyderX2 (0x085C:0x0A0A)");
            ui.add_space(20.0);
            if ui.button("INITIALIZE CALIBRATION").clicked() {
                self.calibrating = true;
                // Hook to measurement pipeline here
            }
        });
    }
}
