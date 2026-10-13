//! The document as a workspace of parts (PartDesign Bodies) and assemblies (`.fcrs` v2).
//!
//! The active part's `Document` lives in `App::doc` while its tab is open (all Part Design
//! code works on it); its entry in `App::ws` then holds an empty placeholder. While an
//! assembly is open every part document is in `ws` (the solver resolves geometry there).
//! Undo snapshots are the whole workspace plus the open tab.

use super::*;
use fc_assembly::{AssemblyId, PartId, Workspace};

/// What the 3D view shows: one part (Part Design) or one assembly.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tab {
    Part(PartId),
    Assembly(AssemblyId),
}

impl Tab {
    fn encode(self) -> String {
        match self {
            Tab::Part(p) => format!("P {p}"),
            Tab::Assembly(a) => format!("A {a}"),
        }
    }
    fn decode(s: &str) -> Option<Tab> {
        let (k, n) = s.split_once(' ')?;
        let n = n.trim().parse().ok()?;
        match k {
            "P" => Some(Tab::Part(n)),
            "A" => Some(Tab::Assembly(n)),
            _ => None,
        }
    }
}

/// A workspace with one empty part, like a new FreeCAD document with a Body.
pub fn fresh_workspace() -> (Workspace, PartId) {
    let mut ws = Workspace::new();
    let p = ws.add_part("Body", Document::new());
    (ws, p)
}

impl App {
    /// Move the active part's document into the workspace (no-op if it is there).
    pub(super) fn park(&mut self) {
        if self.doc_in_ws {
            return;
        }
        if let Tab::Part(p) = self.tab {
            if let Some(e) = self.ws.part_mut(p) {
                std::mem::swap(&mut e.doc, &mut self.doc);
            }
        }
        self.doc_in_ws = true;
    }

    /// Take the open part's document out of the workspace into `self.doc`.
    pub(super) fn unpark(&mut self) {
        if !self.doc_in_ws {
            return;
        }
        if let Tab::Part(p) = self.tab {
            if let Some(e) = self.ws.part_mut(p) {
                std::mem::swap(&mut e.doc, &mut self.doc);
                self.doc_in_ws = false;
            }
        }
    }

    pub(super) fn active_part(&self) -> Option<PartId> {
        match self.tab {
            Tab::Part(p) => Some(p),
            Tab::Assembly(_) => None,
        }
    }

    pub(super) fn active_assembly(&self) -> Option<AssemblyId> {
        match self.tab {
            Tab::Assembly(a) => Some(a),
            Tab::Part(_) => None,
        }
    }

    /// Name of a part (the active one's included).
    pub(super) fn part_name(&self, p: PartId) -> String {
        self.ws.part(p).map(|e| e.name.clone()).unwrap_or_else(|| "?".into())
    }

    /// Undo snapshot: the open tab and the whole workspace.
    pub(super) fn state_json(&mut self) -> String {
        let was = self.doc_in_ws;
        self.park();
        let s = format!("{}\n{}\n{}", self.tab.encode(), serde_json::to_string(&self.ws).expect("workspace serializes"), self.fem_json());
        if !was {
            self.unpark();
        }
        s
    }

    /// Bring back a snapshot. Parts whose definition did not change keep their computed
    /// shapes (no recompute), the others are recomputed.
    pub(super) fn restore(&mut self, json: &str) {
        let mut it = json.splitn(3, '\n');
        let (Some(tab), Some(body)) = (it.next(), it.next()) else { return };
        let fem = it.next().unwrap_or("").to_string();
        let (Some(tab), Ok(mut ws)) = (Tab::decode(tab), Workspace::from_json(body)) else { return };
        self.park();
        let mut old: HashMap<PartId, Document> = self.ws.parts.drain(..).map(|p| (p.id, p.doc)).collect();
        for p in &mut ws.parts {
            match old.remove(&p.id) {
                Some(o) if o.to_json() == p.doc.to_json() => p.doc = o,
                _ => {
                    p.doc.recompute();
                    self.asm.meshes.remove(&p.id);
                }
            }
        }
        self.ws = ws;
        self.tab = tab;
        self.unpark();
        self.fem_restore_json(&fem);
        self.tree_sel.retain(|id| self.doc.get(*id).is_some());
        self.asm.prune(&self.ws, self.active_assembly());
        self.fields.clear();
        self.solve_active(false);
        self.rebuild_scene();
    }

    /// Replace everything with a loaded workspace (Open, New, reload).
    pub(super) fn load_workspace(&mut self, mut ws: Workspace) {
        ws.recompute();
        if ws.parts.is_empty() {
            ws.add_part("Body", Document::new());
        }
        self.tab = match ws.assemblies.first() {
            Some(a) if ws.parts.iter().all(|p| p.doc.features.is_empty()) || ws.parts.len() > 1 => Tab::Assembly(a.id),
            _ => Tab::Part(ws.parts[0].id),
        };
        self.ws = ws;
        self.doc = Document::new();
        self.doc_in_ws = true;
        self.unpark();
        self.asm = assembly::AsmState::default();
        self.fem = fem::FemState { on: self.fem.on && self.active_part().is_some(), ..Default::default() };
        self.tree_sel.clear();
        self.origin_sel = None;
        self.selection.clear();
        self.hidden_sketches.clear();
        self.vp.set_selection(vec![]);
        self.solve_active(false);
        self.rebuild_scene();
        // the assembly's Tasks panel is always open: fit into the rest of the view
        self.vp.right_inset = if self.active_assembly().is_some() { tasks::TASK_W } else { 0.0 };
        self.vp.fit_all();
    }

    /// What Save writes: a single-part workspace stays a plain v1 body document (readable by
    /// older builds and by Calc Canvas); anything else is a v2 workspace.
    pub(super) fn save_json(&mut self) -> String {
        let was = self.doc_in_ws;
        self.park();
        let s = if self.ws.parts.len() == 1 && self.ws.assemblies.is_empty() { self.ws.parts[0].doc.to_json() } else { self.ws.to_json() };
        // FEM analyses ride along as an extra key (older builds ignore it)
        let s = self.fem_save_into(s);
        if !was {
            self.unpark();
        }
        s
    }

    /// Open a part or an assembly in the 3D view.
    pub(super) fn switch_tab(&mut self, t: Tab) {
        if t == self.tab || !self.idle() {
            return;
        }
        if let Tab::Part(p) = self.tab {
            self.last_part = Some(p);
        }
        self.park();
        self.tab = t;
        self.unpark();
        if matches!(t, Tab::Assembly(_)) {
            self.fem.on = false;
        }
        self.fem.sel = None;
        self.tree_sel.clear();
        self.origin_sel = None;
        self.selection.clear();
        self.hidden_sketches.clear();
        self.asm.sel.clear();
        self.vp.set_selection(vec![]);
        self.vp.force_hover(None);
        if let Tab::Assembly(_) = t {
            // parts may have changed since the last solve
            self.asm.meshes.clear();
            self.solve_active(false);
        }
        self.rebuild_scene();
        self.vp.right_inset = if self.active_assembly().is_some() { tasks::TASK_W } else { 0.0 };
        self.vp.fit_all();
        self.status = match t {
            Tab::Part(p) => format!("Part Design: {}", self.part_name(p)),
            Tab::Assembly(a) => format!("Assembly: {}", self.ws.assembly(a).map(|x| x.name.clone()).unwrap_or_default()),
        };
    }

    pub(super) fn unique_part_name(&self, base: &str) -> String {
        let taken = |n: &str| self.ws.parts.iter().any(|p| p.name == n) || self.ws.assemblies.iter().any(|a| a.name == n);
        if !taken(base) {
            return base.to_string();
        }
        (1..).map(|k| format!("{base}{k:03}")).find(|n| !taken(n)).unwrap()
    }

    /// A new empty part (Body) in the document, opened in Part Design.
    pub(super) fn new_part(&mut self) {
        if !self.idle() {
            return;
        }
        self.snapshot();
        let name = self.unique_part_name("Body");
        let id = self.ws.add_part(&name, Document::new());
        self.dirty = true;
        self.switch_tab(Tab::Part(id));
        self.status = format!("Created {name}");
    }

    /// FreeCAD's "Create Assembly": a new assembly, opened in the Assembly workbench.
    pub(super) fn new_assembly(&mut self) {
        if !self.idle() {
            return;
        }
        self.snapshot();
        let name = self.unique_part_name("Assembly");
        let id = self.ws.add_assembly(&name);
        self.dirty = true;
        self.switch_tab(Tab::Assembly(id));
        self.status = format!("Created {name}: insert parts with Insert Component");
    }

    /// Workbench switcher: Part Design opens a part, Assembly opens (or creates) an assembly.
    pub(super) fn switch_workbench(&mut self, assembly: bool) {
        self.fem_leave();
        if assembly {
            match self.ws.assemblies.first().map(|a| a.id) {
                Some(a) => self.switch_tab(Tab::Assembly(a)),
                None => self.new_assembly(),
            }
        } else if self.active_part().is_none() {
            let p = self.last_part.filter(|p| self.ws.part(*p).is_some()).or(self.ws.parts.first().map(|p| p.id));
            if let Some(p) = p {
                self.switch_tab(Tab::Part(p));
            }
        }
    }

    /// Import a FreeCAD assembly file: its assemblies, and the PartDesign bodies it uses.
    pub(super) fn import_fcstd_assembly(&mut self, p: &std::path::Path) -> Option<Result<Workspace, String>> {
        match fc_assembly::fcstd::import(p) {
            Ok((ws, notes)) => {
                let shown: Vec<&String> = notes.iter().filter(|n| !n.contains("guessed")).collect();
                if !shown.is_empty() {
                    let list: Vec<String> = shown.iter().take(12).map(|s| s.to_string()).collect();
                    self.dialog = Some(Dialog::Message(format!("Imported the FreeCAD assembly with notes:\n{}", list.join("\n"))));
                }
                Some(Ok(ws))
            }
            Err(fc_assembly::fcstd::FcstdError::NoAssembly) => None,
            Err(e) => Some(Err(e.to_string())),
        }
    }
}
