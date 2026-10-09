//! `FCRS_SKETCH_SCRIPT=<file>`: scripted input for tests and screenshots. One line runs per
//! frame; clicks go through the same handlers as the real pointer (sketch coordinates are
//! projected to the screen, so snapping uses the real pixel tolerances).
//!
//! Syntax (one command per line, `#` comments; geometry/constraint indices are 0-based):
//! ```text
//! clear                          delete all geometry and constraints
//! tool select|point|line|polyline|rect|crect|circle|arc|arc3|slot
//! move X Y | click X Y | rclick X Y | dclick X Y        (sketch coordinates)
//! dclick label C                 double-click the label/icon of constraint C
//! key Escape|Delete|Enter|<letter>[+cmd][+shift]        e.g. "key g" then "key l"
//! select clear | edge G | point G start|end|mid | root | haxis | vaxis | constraint C
//! constraint <coincident|pointonobject|horizontal|vertical|parallel|perpendicular|tangent|
//!             equal|symmetric|block|lock|distancex|distancey|distance|radius|diameter|angle>
//! dimension <kind> <value or =formula> [name=<name>]  (constraint + fill the popup + OK)
//! popup value <text> | popup name <n> | popup reference on|off | popup ok | popup cancel
//! drag point G start|end|mid X Y | drag edge G X Y | drag label C X Y
//! toggle driving | toggle construction
//! rename C <name> | setvalue C <text>     (panel property editor path)
//! delete | delete constraint C | delete edge G | freeze    (freeze = "delete and keep values")
//! param <name> <expr>                      document parameter
//! undo | redo | fit | wait N | grid on|off | auto on|off | constraints on|off
//! expect dof N | expect status fully|under|conflict|redundant | expect constraints N
//! expect geometry N | expect value C <number> | expect driving C true|false
//! print                                    dump the sketch state to stderr
//! ```
//! Results go to stderr as `[sketch-script] ...`; failed `expect`s print `FAIL`.

use super::{Drag, Ev, Proj, Sel, SketchEditor, Tool};
use crate::viewport::Viewport;
use eframe::egui;
use fc_part::params::OnUsers;
use fc_part::Document;
use fc_sketch::{PointPos, PointRef, Vec2, H_AXIS, V_AXIS};

pub struct Script {
    lines: Vec<String>,
    pc: usize,
    wait: u32,
    settle: u32,
    pub failures: usize,
}

impl Script {
    pub fn load(path: &str) -> Option<Script> {
        let s = std::fs::read_to_string(path).map_err(|e| eprintln!("[sketch-script] cannot read {path}: {e}")).ok()?;
        let lines = s.lines().map(|l| l.split('#').next().unwrap_or("").trim().to_string()).filter(|l| !l.is_empty()).collect();
        Some(Script { lines, pc: 0, wait: 2, settle: 4, failures: 0 })
    }
    pub fn busy(&self) -> bool {
        self.pc < self.lines.len() || self.settle > 0
    }
}

fn pos_of(s: &str) -> Option<PointPos> {
    Some(match s {
        "start" | "s" => PointPos::Start,
        "end" | "e" => PointPos::End,
        "mid" | "center" | "m" => PointPos::Mid,
        "edge" => PointPos::Edge,
        _ => return None,
    })
}

fn key_of(s: &str) -> Option<(egui::Key, egui::Modifiers)> {
    let mut m = egui::Modifiers::NONE;
    let mut parts = s.split('+');
    let k = parts.next()?;
    for p in parts {
        match p {
            "cmd" | "ctrl" => {
                m.command = true;
                m.ctrl = true;
            }
            "shift" => m.shift = true,
            _ => {}
        }
    }
    let key = match k.to_ascii_lowercase().as_str() {
        "escape" | "esc" => egui::Key::Escape,
        "delete" | "del" => egui::Key::Delete,
        "backspace" => egui::Key::Backspace,
        "enter" | "return" => egui::Key::Enter,
        "3" => egui::Key::Num3,
        other => egui::Key::from_name(&other.to_ascii_uppercase())?,
    };
    Some((key, m))
}

impl SketchEditor {
    pub(crate) fn script_step(&mut self, doc: &mut Document, vp: &mut Viewport, ctx: &egui::Context) {
        let Some(sc) = self.script.as_mut() else { return };
        ctx.request_repaint();
        if sc.wait > 0 {
            sc.wait -= 1;
            return;
        }
        if sc.pc >= sc.lines.len() {
            if sc.settle > 0 {
                sc.settle -= 1;
                if sc.settle == 0 {
                    eprintln!("[sketch-script] done ({} failure(s))", sc.failures);
                }
            }
            return;
        }
        let line = sc.lines[sc.pc].clone();
        sc.pc += 1;
        let w: Vec<&str> = line.split_whitespace().collect();
        if w[0] == "fit" {
            self.fit_view(doc, vp);
            return;
        }
        let res = {
            let pj = Proj { vp, pl: self.placement };
            self.script_cmd(doc, &pj, &w, &line)
        };
        match res {
            Ok(Some(msg)) => eprintln!("[sketch-script] {line} -> {msg}"),
            Ok(None) => eprintln!("[sketch-script] {line}"),
            Err(e) => {
                eprintln!("[sketch-script] {line} -> ERROR {e}");
                if let Some(sc) = self.script.as_mut() {
                    sc.failures += 1;
                }
            }
        }
    }

    fn script_cmd(&mut self, doc: &mut Document, pj: &Proj, w: &[&str], line: &str) -> Result<Option<String>, String> {
        let num = |i: usize| -> Result<f64, String> { w.get(i).ok_or("missing number")?.parse::<f64>().map_err(|e| e.to_string()) };
        let idx = |i: usize| -> Result<usize, String> { w.get(i).ok_or("missing index")?.parse::<usize>().map_err(|e| e.to_string()) };
        let at = |i: usize| -> Result<egui::Pos2, String> { Ok(pj.s(Vec2::new(num(i)?, num(i + 1)?))) };
        let rest = |i: usize| -> String { line.split_whitespace().skip(i).collect::<Vec<_>>().join(" ") };
        match w[0] {
            "clear" => {
                let n = self.sk(doc).geometry.len();
                let cons: Vec<usize> = (0..self.sk(doc).constraints.len()).collect();
                self.delete_items(doc, cons, (0..n).collect(), OnUsers::Freeze);
            }
            "tool" => self.set_tool(Tool::parse(w.get(1).ok_or("missing tool")?).ok_or("unknown tool")?),
            "move" => {
                let p = at(1)?;
                self.virtual_cursor = Some(p);
                self.handle_event(doc, pj, Ev::Move(p));
            }
            "click" => {
                let p = at(1)?;
                self.virtual_cursor = Some(p);
                self.handle_event(doc, pj, Ev::Move(p));
                if self.tool.tool == Tool::Select {
                    self.handle_event(doc, pj, Ev::Click(p));
                } else {
                    self.handle_event(doc, pj, Ev::Press(p));
                }
            }
            "rclick" => {
                let p = at(1)?;
                self.handle_event(doc, pj, Ev::RightClick(p));
            }
            "dclick" => {
                let p = if w.get(1) == Some(&"label") {
                    let ci = idx(2)?;
                    self.hits.iter().find(|(c, _)| *c == ci).map(|(_, r)| r.center()).ok_or("label not visible")?
                } else {
                    at(1)?
                };
                self.handle_event(doc, pj, Ev::DoubleClick(p));
            }
            "key" => {
                let (k, m) = key_of(w.get(1).ok_or("missing key")?).ok_or("unknown key")?;
                if self.dim_edit.is_some() {
                    match k {
                        egui::Key::Enter => {
                            self.commit_dim_edit(doc);
                        }
                        egui::Key::Escape => self.cancel_dim_edit(doc),
                        _ => {}
                    }
                } else {
                    self.handle_event(doc, pj, Ev::Key(k, m));
                }
            }
            "select" => {
                let what = *w.get(1).ok_or("missing selection")?;
                let s = match what {
                    "clear" => {
                        self.sel.clear();
                        return Ok(None);
                    }
                    "edge" => Sel::Edge(idx(2)?),
                    "point" => Sel::Point(PointRef::new(idx(2)?, pos_of(w.get(3).ok_or("missing pos")?).ok_or("bad pos")?)),
                    "root" => Sel::Point(PointRef::root()),
                    "haxis" => Sel::Edge(H_AXIS),
                    "vaxis" => Sel::Edge(V_AXIS),
                    "constraint" => Sel::Constraint(idx(2)?),
                    _ => return Err("bad selection".into()),
                };
                if !self.sel.contains(&s) {
                    self.sel.push(s);
                }
            }
            "constraint" => {
                let c = super::Cmd::parse(w.get(1).ok_or("missing constraint")?).ok_or("unknown constraint")?;
                self.run_cmd(doc, c)?;
            }
            "dimension" => {
                let c = super::Cmd::parse(w.get(1).ok_or("missing dimension")?).ok_or("unknown dimension")?;
                let mut text = rest(2);
                let mut name = None;
                if let Some(i) = text.find("name=") {
                    name = Some(text[i + 5..].trim().to_string());
                    text = text[..i].trim().to_string();
                }
                self.run_cmd(doc, c)?;
                let de = self.dim_edit.as_mut().ok_or("no datum popup opened")?;
                if !text.is_empty() {
                    de.value = text;
                }
                if let Some(n) = name {
                    de.name = n;
                }
                if !self.commit_dim_edit(doc) {
                    let e = self.dim_edit.as_ref().and_then(|d| d.error.clone()).unwrap_or_default();
                    self.cancel_dim_edit(doc);
                    return Err(e);
                }
            }
            "popup" => {
                let de = self.dim_edit.as_mut().ok_or("no popup open")?;
                match *w.get(1).ok_or("missing popup command")? {
                    "value" => de.value = rest(2),
                    "name" => de.name = rest(2),
                    "reference" => de.reference = w.get(2) == Some(&"on"),
                    "ok" => {
                        if !self.commit_dim_edit(doc) {
                            return Err(self.dim_edit.as_ref().and_then(|d| d.error.clone()).unwrap_or_default());
                        }
                    }
                    "cancel" => self.cancel_dim_edit(doc),
                    _ => return Err("bad popup command".into()),
                }
                return Ok(None);
            }
            "drag" => {
                let (pr, xi) = match *w.get(1).ok_or("missing drag target")? {
                    "point" => (PointRef::new(idx(2)?, pos_of(w.get(3).ok_or("missing pos")?).ok_or("bad pos")?), 4),
                    "edge" => (PointRef::edge(idx(2)?), 3),
                    "label" => {
                        let ci = idx(2)?;
                        let target = Vec2::new(num(3)?, num(4)?);
                        if let Some(f) = self.dim_frame(self.sk(doc), ci) {
                            self.label_off.insert(ci, f.param_from_cursor(target));
                        }
                        return Ok(None);
                    }
                    _ => return Err("bad drag target".into()),
                };
                let target = Vec2::new(num(xi)?, num(xi + 1)?);
                let from = self.sk(doc).try_point(pr).ok_or("no such point")?;
                self.push_undo(doc);
                self.drag = Drag::Geo { pr, offset: Vec2::ZERO };
                for i in 1..=10 {
                    let p = from.lerp(target, i as f64 / 10.0);
                    self.handle_event(doc, pj, Ev::DragMove(pj.s(p)));
                }
                self.drag = Drag::None;
                self.resolve(doc);
                let now = self.sk(doc).try_point(pr).unwrap_or(from);
                return Ok(Some(format!("point now at ({:.3}, {:.3})", now.x, now.y)));
            }
            "toggle" => match *w.get(1).ok_or("missing toggle")? {
                "driving" => self.toggle_driving(doc),
                "construction" => self.toggle_construction(doc),
                _ => return Err("bad toggle".into()),
            },
            "rename" => {
                let ci = idx(1)?;
                self.push_undo(doc);
                let v = String::new();
                self.apply_dimension(doc, ci, &v, Some(&rest(2)))?;
            }
            "setvalue" => {
                let ci = idx(1)?;
                self.push_undo(doc);
                self.apply_dimension(doc, ci, &rest(2), None)?;
            }
            "delete" => match w.get(1).copied() {
                None => self.delete_selection(doc, OnUsers::Refuse),
                Some("constraint") => self.delete_items(doc, vec![idx(2)?], vec![], OnUsers::Refuse),
                Some("edge") => self.delete_items(doc, vec![], vec![idx(2)?], OnUsers::Refuse),
                _ => return Err("bad delete".into()),
            },
            "freeze" => {
                let pd = self.pending_delete.take().ok_or("no pending delete")?;
                self.delete_items(doc, pd.constraints, pd.geometry, OnUsers::Freeze);
            }
            "param" => {
                doc.set_parameter(w.get(1).ok_or("missing name")?, &rest(2)).map_err(|e| e.to_string())?;
                self.resolve(doc);
            }
            "undo" => self.undo_step(doc),
            "redo" => self.redo_step(doc),
            "wait" => {
                let n = idx(1)? as u32;
                if let Some(sc) = self.script.as_mut() {
                    sc.wait = n;
                }
            }
            "grid" => self.grid = w.get(1) == Some(&"on"),
            "auto" => self.auto_constraints = w.get(1) == Some(&"on"),
            "constraints" => self.show_constraints = w.get(1) == Some(&"on"),
            "mode" => {
                if w.get(1) == Some(&"construction") {
                    self.construction = w.get(2) != Some(&"off");
                }
            }
            "expect" => return self.expect(doc, w),
            "print" => return Ok(Some(self.dump(doc))),
            _ => return Err(format!("unknown command '{}'", w[0])),
        }
        Ok(self.pending_delete.as_ref().map(|p| format!("delete refused; used by: {}", p.users.join(", "))))
    }

    fn expect(&mut self, doc: &Document, w: &[&str]) -> Result<Option<String>, String> {
        let r = self.report.clone().ok_or("no solve yet")?;
        let sk = self.sk(doc);
        let (ok, got) = match *w.get(1).ok_or("missing expectation")? {
            "dof" => {
                let n: usize = w.get(2).ok_or("missing")?.parse().map_err(|_| "bad number")?;
                (r.dof == n, format!("dof {}", r.dof))
            }
            "status" => {
                let s = if !r.conflicting.is_empty() {
                    "conflict"
                } else if !r.redundant.is_empty() {
                    "redundant"
                } else if r.dof == 0 && r.is_ok() {
                    "fully"
                } else {
                    "under"
                };
                (Some(&s) == w.get(2), format!("status {s} (conflicting {:?}, redundant {:?})", r.conflicting, r.redundant))
            }
            "constraints" => {
                let n: usize = w.get(2).ok_or("missing")?.parse().map_err(|_| "bad number")?;
                (sk.constraints.len() == n, format!("{} constraints", sk.constraints.len()))
            }
            "geometry" => {
                let n: usize = w.get(2).ok_or("missing")?.parse().map_err(|_| "bad number")?;
                (sk.geometry.len() == n, format!("{} geometries", sk.geometry.len()))
            }
            "value" => {
                let ci: usize = w.get(2).ok_or("missing")?.parse().map_err(|_| "bad index")?;
                let v: f64 = w.get(3).ok_or("missing")?.parse().map_err(|_| "bad number")?;
                let got = sk.constraints.get(ci).and_then(|c| super::geom::display_value(&c.kind)).ok_or("not a dimension")?;
                ((got - v).abs() < 1e-6 * v.abs().max(1.0), format!("value {got}"))
            }
            "driving" => {
                let ci: usize = w.get(2).ok_or("missing")?.parse().map_err(|_| "bad index")?;
                let want = w.get(3) == Some(&"true");
                let got = sk.constraints.get(ci).ok_or("no such constraint")?.driving;
                (got == want, format!("driving {got}"))
            }
            _ => return Err("unknown expectation".into()),
        };
        if ok {
            Ok(Some(format!("PASS ({got})")))
        } else {
            if let Some(sc) = self.script.as_mut() {
                sc.failures += 1;
            }
            Ok(Some(format!("FAIL (got {got})")))
        }
    }

    fn dump(&self, doc: &Document) -> String {
        let sk = self.sk(doc);
        let mut s = String::new();
        if let Some(r) = &self.report {
            s.push_str(&format!("status {:?} dof {} conflicting {:?} redundant {:?}\n", r.status, r.dof, r.conflicting, r.redundant));
        }
        for (i, g) in sk.geometry.iter().enumerate() {
            s.push_str(&format!("  g{i}: {:?}{}\n", g.curve, if g.construction { " (construction)" } else { "" }));
        }
        for (i, c) in sk.constraints.iter().enumerate() {
            s.push_str(&format!("  c{i}: {:?} driving={} name={:?}\n", c.kind, c.driving, c.name));
        }
        for (k, v) in &doc.expressions {
            s.push_str(&format!("  expr {k} = {v}\n"));
        }
        s
    }
}
