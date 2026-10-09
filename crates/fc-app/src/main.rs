mod app;
mod camera;
mod demo;
mod scene;
mod sketcher;
mod gpu;
mod viewport;

use std::path::PathBuf;

fn main() -> eframe::Result {
    let shot = std::env::var("FCRS_SCREENSHOT").ok().map(PathBuf::from);
    let (w, h) = std::env::var("FCRS_SIZE")
        .ok()
        .and_then(|s| {
            let (a, b) = s.split_once('x')?;
            Some((a.parse::<f32>().ok()?, b.parse::<f32>().ok()?))
        })
        .unwrap_or((1280.0, 800.0));
    let options = eframe::NativeOptions {
        viewport: egui_viewport(w, h, shot.is_none()),
        multisampling: 4,
        depth_buffer: 24,
        ..Default::default()
    };
    eframe::run_native(
        "freecad-rs",
        options,
        Box::new(move |cc| Ok(Box::new(app::App::new(cc, shot)))),
    )
}

fn egui_viewport(w: f32, h: f32, active: bool) -> eframe::egui::ViewportBuilder {
    eframe::egui::ViewportBuilder::default()
        .with_inner_size([w, h])
        .with_title("freecad-rs")
        .with_active(active)
}
