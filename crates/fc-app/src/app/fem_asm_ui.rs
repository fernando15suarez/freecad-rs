//! FEM on an assembly (FreeCAD analyses assemblies as one shape with several solids): the
//! setup task (bonded via BooleanFragments or separate parts, a material per part, contact
//! detection that creates ties or contact) and the Tasks panel while no task is open.

use super::fem::*;
use super::*;
use crate::theme;
use crate::widgets::{self, task_box, task_row};
use egui::{Color32, RichText, Stroke};
use fc_fem::{ContactBehavior, ContactKind, InteractionKind, Joining, Material};

impl App {
    /// "Part.FaceN" of a face of the analysed shape (its own face number in the part).
    fn fem_face_label(&self, a: usize, part: usize, global: usize) -> String {
        let d = &self.fem.docs[a];
        let Some(p) = d.analysis.parts.get(part) else { return format!("Face{global}") };
        match p.faces.iter().find(|f| f.1 == global) {
            Some((local, _)) => format!("{}.Face{local}", p.name),
            None => format!("{}.Face{global}", p.name),
        }
    }

    pub(super) fn fem_asm_setup_task(&mut self, ui: &mut egui::Ui, a: usize) {
        let (cancel, ok) = self.task_buttons(ui, ("task:cancel", "Cancel"), Some(("task:ok", "OK")));
        let Some(cfg) = self.fem.docs[a].asm.clone() else {
            self.fem.task = None;
            return;
        };
        let Some(FemTask::AsmSetup { contacts, tol, .. }) = &self.fem.task else { return };
        let (contacts, mut tol) = (contacts.clone(), *tol);
        let lib: Vec<String> = Material::library().into_iter().map(|m| m.name).collect();
        let mut new = cfg.clone();
        let (mut detect, mut add): (bool, Option<(usize, bool)>) = (false, None);
        let mut remove = None;
        let inter: Vec<(String, String, Vec<usize>, Vec<usize>)> = self.fem.docs[a]
            .analysis
            .interactions
            .iter()
            .map(|i| {
                let k = if matches!(i.kind, InteractionKind::Tie { .. }) { "Tie" } else { "Contact" };
                (i.name.clone(), k.to_string(), i.slave.iter().map(|r| r.index).collect(), i.master.iter().map(|r| r.index).collect())
            })
            .collect();
        let error = self.fem.docs[a].rt.error.clone();
        egui::ScrollArea::vertical().id_salt("fem_task_scroll").auto_shrink([false, true]).show(ui, |ui| {
            if let Some(e) = &error {
                egui::Frame::new().fill(Color32::from_rgb(70, 32, 30)).corner_radius(4.0).inner_margin(8.0).show(ui, |ui| {
                    ui.set_width(ui.available_width());
                    ui.label(RichText::new(e).color(Color32::from_rgb(255, 190, 180)));
                });
                ui.add_space(8.0);
            }
            task_box(ui, "fem_join", "Part_BooleanFragments", "Assembly analysis", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                let bonded = matches!(new.joining, Joining::Fragments { .. });
                let mut j = bonded;
                let r = ui.radio_value(&mut j, true, "Bonded parts (BooleanFragments)").on_hover_text("Touching parts share their common faces and one conformal mesh: they are glued together, FreeCAD's usual way to analyse assemblies");
                widgets::reg("fem:join:bonded", r.rect);
                let r = ui.radio_value(&mut j, false, "Separate parts (ties or contact)").on_hover_text("Each part is meshed on its own; join touching faces with ties (bonded) or contact (they may separate or slide)");
                widgets::reg("fem:join:separate", r.rect);
                if j != bonded {
                    new.joining = if j { Joining::Fragments { fuzzy: 0.0 } } else { Joining::Separate };
                }
            });
            task_box(ui, "fem_pmat", "FEM_MaterialSolid", "Materials", |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                for (i, (name, m)) in new.materials.iter_mut().enumerate() {
                    let mut cur = m.name.clone();
                    let mut names = lib.clone();
                    if !names.contains(&cur) {
                        names.push(cur.clone());
                    }
                    let opts: Vec<(String, &str)> = names.iter().map(|n| (n.clone(), n.as_str())).collect();
                    let label = name.clone();
                    ui.horizontal(|ui| {
                        let (r, _) = ui.allocate_exact_size(egui::vec2(10.0, 10.0), egui::Sense::hover());
                        let c = part_color(i);
                        ui.painter().rect_filled(r, 2.0, Color32::from_rgb((c[0] * 255.0) as u8, (c[1] * 255.0) as u8, (c[2] * 255.0) as u8));
                        ui.label(RichText::new(&label).color(theme::TEXT));
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if widgets::combo(ui, &format!("fem_pmat:{label}"), &mut cur, &opts, 150.0) {
                                if let Some(x) = Material::by_name(&cur) {
                                    *m = x;
                                }
                            }
                        });
                    });
                }
                info_text2(ui, "From the parts' material (Bill of Materials); a change applies to this analysis.");
            });
            task_box(ui, "fem_contacts", "FEM_ConstraintContact", "Contacts", |ui| {
                ui.spacing_mut().item_spacing.y = 6.0;
                task_row(ui, "Tolerance", |ui| super::fem_ui::qty(ui, "contact_tol", &mut tol, "mm", true));
                detect = widgets::push_button(ui, "fem:detect", "Detect contacts", ui.available_width() - 4.0, false).on_hover_text("Find faces of different parts that touch: planes facing each other, pins in holes").clicked();
                match &contacts {
                    None => info_text2(ui, "Find touching faces, then make them ties or contact."),
                    Some(Err(e)) => {
                        ui.label(RichText::new(e).color(theme::ERROR));
                    }
                    Some(Ok(list)) if list.is_empty() => info_text2(ui, "No touching faces found within the tolerance."),
                    Some(Ok(list)) => {
                        egui::Frame::new().fill(theme::INPUT).stroke(Stroke::new(1.0, theme::BORDER)).corner_radius(3.0).inner_margin(4.0).show(ui, |ui| {
                            ui.set_width(ui.available_width());
                            for (i, c) in list.iter().enumerate() {
                                let sl = self.fem_face_label(a, c.slave_part, c.slave_face);
                                let ml = self.fem_face_label(a, c.master_part, c.master_face);
                                ui.label(RichText::new(format!("{sl} ↔ {ml}")).color(theme::TEXT));
                                let kind = if c.kind == ContactKind::Cylindrical { "cylinders" } else { "planes" };
                                let made = inter.iter().find(|x| x.2.contains(&c.slave_face) && x.3.contains(&c.master_face)).map(|x| x.1.clone());
                                ui.horizontal(|ui| {
                                    ui.label(RichText::new(format!("{} mm², {kind}", super::fem::sig(c.area))).color(theme::TEXT_DIM).small());
                                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                        if c.shared {
                                            ui.label(RichText::new("bonded (shared face)").color(theme::GREEN).small());
                                        } else if let Some(k) = &made {
                                            ui.label(RichText::new(format!("{k} made")).color(theme::GREEN).small());
                                        } else {
                                            let r = ui.small_button("Contact").on_hover_text("Penalty contact: the faces may separate and slide (CalculiX *CONTACT PAIR)");
                                            widgets::reg(format!("fem:contact:{i}"), r.rect);
                                            if r.clicked() {
                                                add = Some((i, false));
                                            }
                                            let r = ui.small_button("Tie").on_hover_text("Bond the faces (CalculiX *TIE)");
                                            widgets::reg(format!("fem:tie:{i}"), r.rect);
                                            if r.clicked() {
                                                add = Some((i, true));
                                            }
                                        }
                                    });
                                });
                                ui.add_space(2.0);
                            }
                        });
                    }
                }
                if !inter.is_empty() {
                    ui.add_space(4.0);
                    for (i, (name, k, _, _)) in inter.iter().enumerate() {
                        ui.horizontal(|ui| {
                            ui.add(crate::icons::image(ui.ctx(), if k == "Tie" { "FEM_ConstraintTie" } else { "FEM_ConstraintContact" }, 16.0));
                            ui.label(RichText::new(name).color(theme::TEXT));
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                let b = ui.add(egui::Button::image(crate::icons::image(ui.ctx(), "edit-delete", 14.0)).frame(false)).on_hover_text("Delete");
                                if b.clicked() {
                                    remove = Some(i);
                                }
                            });
                        });
                    }
                }
            });
        });
        if let Some(FemTask::AsmSetup { tol: t, .. }) = self.fem.task.as_mut() {
            *t = tol;
        }
        if new != cfg {
            self.fem.docs[a].asm = Some(new.clone());
            self.dirty = true;
            match self.fem_asm_build(a) {
                Ok(()) => self.fem.docs[a].rt.error = None,
                Err(e) => self.fem.docs[a].rt.error = Some(e),
            }
            // ties and contacts belong to the separate-parts mesh; bonded parts need none
            if matches!(new.joining, Joining::Fragments { .. }) != matches!(cfg.joining, Joining::Fragments { .. }) {
                self.fem.docs[a].analysis.interactions.clear();
                let d = &mut self.fem.docs[a].rt;
                d.mesh = None;
                d.surface = None;
                d.results = None;
                d.part_results = None;
                if let Some(FemTask::AsmSetup { contacts, .. }) = self.fem.task.as_mut() {
                    *contacts = None;
                }
            }
            self.rebuild_scene();
        }
        if let Some(i) = remove {
            self.fem.docs[a].analysis.interactions.remove(i);
            self.dirty = true;
        }
        if detect {
            let r = {
                let d = &self.fem.docs[a];
                match &d.rt.shape {
                    Some(sh) => {
                        let mut an = d.analysis.clone();
                        an.shape = fc_fem::ShapeSource::Shape(sh.clone());
                        fc_fem::detect_contacts(&an, tol).map_err(|e| e.to_string())
                    }
                    None => Err("The analysis has no shape".to_string()),
                }
            };
            self.status = match &r {
                Ok(v) => format!("{} touching face pair{}", v.len(), if v.len() == 1 { "" } else { "s" }),
                Err(e) => e.clone(),
            };
            if let Some(FemTask::AsmSetup { contacts, .. }) = self.fem.task.as_mut() {
                *contacts = Some(r);
            }
        }
        if let Some((i, tie)) = add {
            let pair = match &self.fem.task {
                Some(FemTask::AsmSetup { contacts: Some(Ok(v)), .. }) => v.get(i).cloned(),
                _ => None,
            };
            if let Some(p) = pair {
                let (s, m) = (self.fem.docs[a].rt.shape.as_ref().and_then(|sh| fc_fem::SubRef::face_of(sh, p.slave_face)).unwrap_or(fc_fem::SubRef::face(p.slave_face)), self.fem.docs[a].rt.shape.as_ref().and_then(|sh| fc_fem::SubRef::face_of(sh, p.master_face)).unwrap_or(fc_fem::SubRef::face(p.master_face)));
                let kind = if tie {
                    InteractionKind::Tie { tolerance: tol.max(p.gap * 1.5), adjust: false }
                } else {
                    // FreeCAD's ConstraintContact defaults: linear penalty, ADJUST closes small gaps
                    InteractionKind::Contact { behavior: ContactBehavior::Linear, slope: 1e6, adjust: tol.max(p.gap * 1.5), friction: None }
                };
                let name = self.fem.docs[a].analysis.add_interaction(kind, vec![s], vec![m]).name.clone();
                self.dirty = true;
                self.status = format!("Added {name}");
            }
        }
        if ok {
            self.fem_task_ok();
        } else if cancel {
            self.fem_task_cancel();
        }
    }

    /// The Tasks panel of the FEM workbench on an assembly when no task is open.
    pub(super) fn fem_asm_overview(&mut self, ui: &mut egui::Ui) {
        let mut act: Option<super::chrome::Act> = None;
        let cur = self.fem_cur();
        egui::ScrollArea::vertical().id_salt("fem_asm_overview").auto_shrink([false, true]).show(ui, |ui| {
            task_box(ui, "fem_asm", "FEM_Analysis", "Assembly analysis", |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                match cur {
                    None => {
                        ui.label(RichText::new("Analyse the whole assembly: its parts bonded together, or joined by ties and contact, each with its own material.").color(Color32::from_gray(185)));
                        ui.add_space(4.0);
                        if Self::link_row(ui, "taskact:fem_analysis", "FEM_Analysis", "New analysis", "Create an analysis of this assembly") {
                            act = Some(super::chrome::Act::Fem(FemCmd::Analysis));
                        }
                    }
                    Some(a) => {
                        let d = &self.fem.docs[a];
                        ui.label(RichText::new(format!("{}: {} parts, {} constraints, {} ties/contacts", d.analysis.name, d.analysis.parts.len(), d.analysis.constraints.len(), d.analysis.interactions.len())).color(theme::TEXT));
                        let rows = [
                            ("taskact:fem_setup", "Part_BooleanFragments", "Joining, materials, contacts", FemCmd::Material),
                            ("taskact:fem_mesh", "FEM_MeshGmshFromShape", "Mesh", FemCmd::Mesh),
                            ("taskact:fem_run", "FEM_SolverRun", "Run CalculiX", FemCmd::Run),
                            ("taskact:fem_results", "FEM_ResultShow", "Results", FemCmd::Results),
                        ];
                        for (reg, icon, text, c) in rows {
                            if Self::link_row(ui, reg, icon, text, "") {
                                act = Some(super::chrome::Act::Fem(c));
                            }
                        }
                        info_text2(ui, "Constraints: pick faces of the parts with the toolbar's boundary condition and load tools.");
                    }
                }
            });
        });
        if let Some(a) = act {
            self.run(a);
        }
    }
}

fn info_text2(ui: &mut egui::Ui, text: &str) {
    ui.label(RichText::new(text).color(Color32::from_gray(185)));
}
