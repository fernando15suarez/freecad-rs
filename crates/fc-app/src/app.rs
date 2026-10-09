use crate::demo;
use crate::viewport::{Pick, PickKind, StandardView, Viewport};
use eframe::egui;
use std::path::PathBuf;

pub struct App {
    vp: Viewport,
    selection: Vec<Pick>,
    hovered: Option<Pick>,
    status: String,
    /// Test hook: (path, frames rendered)
    screenshot: Option<(PathBuf, u32)>,
}

fn pick_name(vp: &Viewport, p: &Pick) -> String {
    let obj = vp.objects().get(p.object).map(|o| o.name.as_str()).unwrap_or("?");
    let kind = match p.kind {
        PickKind::Face => "Face",
        PickKind::Edge => "Edge",
        PickKind::Vertex => "Vertex",
    };
    format!("{obj}.{kind}{}", p.id + 1)
}

fn parse_pick(s: &str) -> Option<Pick> {
    let (k, id) = s.split_once(':')?;
    let kind = match k {
        "face" => PickKind::Face,
        "edge" => PickKind::Edge,
        _ => return None,
    };
    Some(Pick { object: 0, kind, id: id.parse().ok()?, point: [0.0; 3] })
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, screenshot: Option<PathBuf>) -> Self {
        let mut vp = Viewport::new(cc);
        vp.set_objects(vec![demo::box_with_hole()]);
        let view = match std::env::var("FCRS_VIEW").as_deref() {
            Ok("front") => Some(StandardView::Front),
            Ok("top") => Some(StandardView::Top),
            Ok("right") => Some(StandardView::Right),
            Ok("rear") => Some(StandardView::Rear),
            Ok("bottom") => Some(StandardView::Bottom),
            Ok("left") => Some(StandardView::Left),
            _ => Some(StandardView::Isometric),
        };
        if let Some(v) = view {
            vp.set_standard_view(v);
        }
        if std::env::var_os("FCRS_PERSP").is_some() {
            vp.set_perspective(true);
            vp.fit_all();
        }
        let mut selection = vec![];
        if let Ok(s) = std::env::var("FCRS_SELECT") {
            selection = s.split(',').filter_map(parse_pick).collect();
            vp.set_selection(selection.clone());
        }
        if let Some(h) = std::env::var("FCRS_HOVER").ok().and_then(|s| parse_pick(&s)) {
            vp.force_hover(Some(h));
        }
        App { vp, selection, hovered: None, status: "Ready".into(), screenshot: screenshot.map(|p| (p, 0)) }
    }

    fn log(&mut self, s: &str) {
        println!("[action] {s}");
        self.status = s.to_string();
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        let ctx = &ctx;
        egui::Panel::top("toolbar").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                for name in ["New", "Open", "Save", "Export STEP", "Export STL"] {
                    if ui.button(name).clicked() {
                        self.log(&format!("{name} (not implemented)"));
                    }
                }
                ui.separator();
                for (name, v) in [
                    ("Iso", StandardView::Isometric),
                    ("Front", StandardView::Front),
                    ("Top", StandardView::Top),
                    ("Right", StandardView::Right),
                    ("Rear", StandardView::Rear),
                    ("Bottom", StandardView::Bottom),
                    ("Left", StandardView::Left),
                ] {
                    if ui.button(name).clicked() {
                        self.vp.set_standard_view(v);
                    }
                }
                if ui.button("Fit").clicked() {
                    self.vp.fit_all();
                }
                let mut persp = self.vp.perspective();
                if ui.checkbox(&mut persp, "Perspective").changed() {
                    self.vp.set_perspective(persp);
                    self.vp.fit_all();
                }
                ui.separator();
                for name in ["Sketch", "Pad", "Pocket", "Revolution", "Groove"] {
                    if ui.button(name).clicked() {
                        self.log(&format!("{name} (not implemented)"));
                    }
                }
            });
        });

        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                if let Some(h) = &self.hovered {
                    ui.label(format!(
                        "Preselected: {} ({:.2}, {:.2}, {:.2})",
                        pick_name(&self.vp, h),
                        h.point[0],
                        h.point[1],
                        h.point[2]
                    ));
                } else {
                    ui.label(&self.status);
                }
            });
        });

        egui::Panel::left("model").default_size(240.0).show(ui, |ui| {
            ui.heading("Model");
            egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), "tree_body".into(), true)
                .show_header(ui, |ui| ui.label("Body"))
                .body(|ui| {
                    egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), "tree_sketch".into(), false)
                        .show_header(ui, |ui| ui.label("Sketch"))
                        .body(|ui| {
                            ui.label("Line, Line, Line, Line");
                        });
                    ui.label("Pad");
                });
            ui.separator();
            ui.heading("Properties");
            if self.selection.is_empty() {
                ui.weak("Nothing selected");
            }
            for p in &self.selection {
                ui.label(pick_name(&self.vp, p));
            }
        });

        egui::CentralPanel::default().frame(egui::Frame::NONE).show(ui, |ui| {
            let r = self.vp.ui(ui);
            self.hovered = r.hovered.clone();
            if r.response.clicked() {
                let multi = ctx.input(|i| i.modifiers.ctrl || i.modifiers.command);
                match r.clicked {
                    Some(p) => {
                        if multi {
                            if let Some(i) = self.selection.iter().position(|s| s.kind == p.kind && s.id == p.id && s.object == p.object) {
                                self.selection.remove(i);
                            } else {
                                self.selection.push(p);
                            }
                        } else {
                            self.selection = vec![p];
                        }
                    }
                    None => {
                        if !multi {
                            self.selection.clear();
                        }
                    }
                }
                self.vp.set_selection(self.selection.clone());
                if let Some(p) = self.selection.last() {
                    self.status = format!("Selected: {}", pick_name(&self.vp, p));
                }
            }
        });

        if let Some((path, frames)) = &mut self.screenshot {
            *frames += 1;
            if *frames == 6 {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(Default::default()));
            }
            let shot = ctx.input(|i| {
                i.raw.events.iter().find_map(|e| match e {
                    egui::Event::Screenshot { image, .. } => Some(image.clone()),
                    _ => None,
                })
            });
            if let Some(img) = shot {
                let [w, h] = img.size;
                let bytes: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
                let _ = image::save_buffer(&*path, &bytes, w as u32, h as u32, image::ExtendedColorType::Rgba8);
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint();
        }
    }
}
