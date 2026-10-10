//! FreeCAD-style widgets: toolbar icon buttons, task boxes, the property grid, dock titles
//! and a registry of named widget rects that the input script (`at ui NAME`) can aim at.

use crate::icons;
use crate::theme;
use eframe::egui::{self, Align2, Color32, CornerRadius, Pos2, Rect, RichText, Sense, Stroke, Vec2};
use std::cell::RefCell;
use std::collections::HashMap;

// ----------------------------------------------------------------- ui registry

thread_local! {
    static RECTS: RefCell<HashMap<String, Rect>> = RefCell::new(HashMap::new());
}

/// Remember where widget `name` was drawn this frame (test hook target).
pub fn reg(name: impl Into<String>, rect: Rect) {
    RECTS.with(|r| r.borrow_mut().insert(name.into(), rect));
}

pub fn lookup(name: &str) -> Option<Rect> {
    RECTS.with(|r| r.borrow().get(name).copied())
}

pub fn clear_registry() {
    RECTS.with(|r| r.borrow_mut().clear());
}

// --------------------------------------------------------------------- tooltip

/// Qt-style rich tooltip: bold title, description, shortcut.
pub fn tooltip(ui: &mut egui::Ui, title: &str, desc: &str, shortcut: &str) {
    ui.set_max_width(320.0);
    ui.label(RichText::new(title).font(theme::bold_font(13.0)));
    if !desc.is_empty() {
        ui.label(desc);
    }
    if !shortcut.is_empty() {
        ui.label(RichText::new(format!("Shortcut: {shortcut}")).color(theme::TEXT_DIM).small());
    }
}

// --------------------------------------------------------------------- toolbar

pub const TB_ICON: f32 = 25.0;
pub const TB_BUTTON: f32 = 33.0;

pub struct Action<'a> {
    pub id: &'a str,
    pub icon: &'a str,
    pub title: &'a str,
    pub desc: &'a str,
    pub shortcut: &'a str,
    pub enabled: bool,
    pub checked: bool,
}

impl<'a> Action<'a> {
    pub fn new(id: &'a str, icon: &'a str, title: &'a str, desc: &'a str, shortcut: &'a str) -> Self {
        Action { id, icon, title, desc, shortcut, enabled: true, checked: false }
    }
    pub fn enabled(mut self, e: bool) -> Self {
        self.enabled = e;
        self
    }
    pub fn checked(mut self, c: bool) -> Self {
        self.checked = c;
        self
    }
}

/// Flat icon button like a QToolButton in a toolbar. Returns true when clicked.
pub fn tool_button(ui: &mut egui::Ui, a: &Action) -> bool {
    let r = tool_button_resp(ui, a);
    a.enabled && r.clicked()
}

/// [`tool_button`] returning the response (e.g. to attach a dropdown menu).
pub fn tool_button_resp(ui: &mut egui::Ui, a: &Action) -> egui::Response {
    let (rect, resp) = ui.allocate_exact_size(Vec2::splat(TB_BUTTON), if a.enabled { Sense::click() } else { Sense::hover() });
    reg(format!("tb:{}", a.id), rect);
    let p = ui.painter();
    let hovered = resp.hovered() && a.enabled;
    if a.checked {
        p.rect(rect.shrink(1.0), CornerRadius::same(4), Color32::from_rgb(40, 62, 88), Stroke::new(1.0, theme::ACCENT), egui::StrokeKind::Inside);
    } else if hovered {
        let fill = if resp.is_pointer_button_down_on() { Color32::from_rgb(70, 70, 70) } else { theme::BUTTON_HOVER };
        p.rect(rect.shrink(1.0), CornerRadius::same(4), fill, Stroke::new(1.0, Color32::from_rgb(80, 80, 80)), egui::StrokeKind::Inside);
    }
    let ir = Rect::from_center_size(rect.center(), Vec2::splat(TB_ICON));
    icons::paint_ex(p, a.icon, ir, Color32::WHITE, !a.enabled);
    resp.on_hover_ui(|ui| tooltip(ui, a.title, a.desc, a.shortcut))
}

/// Tool button with a dropdown arrow (QToolButton MenuButtonPopup): the button runs the
/// current variant, the arrow lists all variants. Returns (button clicked, variant chosen).
pub fn split_tool_button(ui: &mut egui::Ui, main: &Action, variants: &[Action]) -> (bool, Option<usize>) {
    let clicked = tool_button(ui, main);
    let (rect, resp) = ui.allocate_exact_size(egui::vec2(11.0, TB_BUTTON), Sense::click());
    reg(format!("tb:{}:menu", main.id), rect);
    if resp.hovered() {
        ui.painter().rect_filled(rect.shrink2(egui::vec2(0.0, 3.0)), 3.0, theme::BUTTON_HOVER);
    }
    let c = rect.center();
    ui.painter().add(egui::Shape::convex_polygon(vec![c + egui::vec2(-3.5, -1.5), c + egui::vec2(3.5, -1.5), c + egui::vec2(0.0, 2.5)], Color32::from_gray(170), Stroke::NONE));
    let mut chosen = None;
    egui::Popup::menu(&resp).show(|ui| {
        ui.set_min_width(200.0);
        for (i, v) in variants.iter().enumerate() {
            let mut b = egui::Button::image_and_text(icons::image(ui.ctx(), v.icon, 20.0), v.title).selected(v.checked);
            if !v.shortcut.is_empty() {
                b = b.shortcut_text(v.shortcut);
            }
            let r = ui.add_enabled(v.enabled, b);
            reg(format!("tb:{}", v.id), r.rect);
            if r.on_hover_text(v.desc).clicked() {
                chosen = Some(i);
            }
        }
    });
    (clicked, chosen)
}

/// Vertical toolbar separator (FreeCAD draws a dotted handle at each toolbar's start).
pub fn tb_separator(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(9.0, TB_BUTTON), Sense::hover());
    let x = rect.center().x;
    ui.painter().line_segment([Pos2::new(x, rect.top() + 5.0), Pos2::new(x, rect.bottom() - 5.0)], Stroke::new(1.0, Color32::from_rgb(60, 60, 60)));
}

/// The dotted grip FreeCAD shows at the start of each toolbar.
pub fn tb_grip(ui: &mut egui::Ui) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(8.0, TB_BUTTON), Sense::hover());
    for i in 0..5 {
        let y = rect.top() + 8.0 + i as f32 * 3.5;
        for dx in [2.5, 5.5] {
            ui.painter().circle_filled(Pos2::new(rect.left() + dx, y), 0.8, Color32::from_rgb(85, 85, 85));
        }
    }
}

// ------------------------------------------------------------------- dock title

/// Dock title ("Model", "Tasks") like a Qt dock widget title bar.
pub fn dock_title(ui: &mut egui::Ui, title: &str) {
    let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), 26.0), Sense::hover());
    ui.painter().text(Pos2::new(rect.left() + 8.0, rect.center().y), Align2::LEFT_CENTER, title, theme::bold_font(13.0), theme::TEXT);
    ui.painter().line_segment([Pos2::new(rect.left() + 4.0, rect.bottom() - 1.0), Pos2::new(rect.right() - 4.0, rect.bottom() - 1.0)], Stroke::new(1.0, theme::GRID_LINE));
    ui.add_space(4.0);
}

// ---------------------------------------------------------------------- buttons

/// A push button of a fixed width, optionally the default ("OK") one.
pub fn push_button(ui: &mut egui::Ui, id: &str, text: &str, width: f32, default: bool) -> egui::Response {
    push_button_enabled(ui, id, text, width, default, true)
}

pub fn push_button_enabled(ui: &mut egui::Ui, id: &str, text: &str, width: f32, default: bool, enabled: bool) -> egui::Response {
    let mut b = egui::Button::new(RichText::new(text).size(14.0)).min_size(egui::vec2(width, 26.0)).corner_radius(4.0);
    if default {
        b = b.fill(theme::OK_BUTTON).stroke(Stroke::new(1.0, Color32::from_rgb(70, 130, 200)));
    }
    let r = ui.add_enabled(enabled, b);
    reg(id.to_string(), r.rect);
    r
}

// --------------------------------------------------------------------- task box

/// Collapsible task box (Gui::TaskView::TaskBox): icon + bold title header, body below.
pub fn task_box<R>(ui: &mut egui::Ui, id: &str, icon: &str, title: &str, add: impl FnOnce(&mut egui::Ui) -> R) -> Option<R> {
    let open_id = egui::Id::new(("taskbox", id));
    let mut open = ui.data(|d| d.get_temp::<bool>(open_id)).unwrap_or(true);
    let width = ui.available_width();
    let (hrect, hresp) = ui.allocate_exact_size(egui::vec2(width, 28.0), Sense::click());
    reg(format!("box:{id}"), hrect);
    let p = ui.painter();
    let cr = if open { CornerRadius { nw: 5, ne: 5, sw: 0, se: 0 } } else { CornerRadius::same(5) };
    p.rect(hrect, cr, if hresp.hovered() { Color32::from_rgb(46, 46, 46) } else { theme::BOX_HEADER }, Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
    let ir = Rect::from_center_size(Pos2::new(hrect.left() + 16.0, hrect.center().y), Vec2::splat(16.0));
    icons::paint(p, icon, ir, Color32::WHITE);
    p.text(Pos2::new(hrect.left() + 30.0, hrect.center().y), Align2::LEFT_CENTER, title, theme::bold_font(12.5), theme::TEXT);
    // collapse arrow
    let c = Pos2::new(hrect.right() - 16.0, hrect.center().y);
    let pts = if open { vec![c + egui::vec2(-5.0, 3.0), c + egui::vec2(5.0, 3.0), c + egui::vec2(0.0, -3.0)] } else { vec![c + egui::vec2(-5.0, -3.0), c + egui::vec2(5.0, -3.0), c + egui::vec2(0.0, 3.0)] };
    p.add(egui::Shape::convex_polygon(pts, Color32::from_rgb(170, 170, 170), Stroke::NONE));
    if hresp.clicked() {
        open = !open;
        ui.data_mut(|d| d.insert_temp(open_id, open));
    }
    if !open {
        ui.add_space(6.0);
        return None;
    }
    let r = egui::Frame::new()
        .fill(theme::BOX_BODY)
        .stroke(Stroke::new(1.0, theme::BORDER))
        .corner_radius(CornerRadius { nw: 0, ne: 0, sw: 5, se: 5 })
        .inner_margin(egui::Margin { left: 10, right: 10, top: 8, bottom: 10 })
        .show(ui, |ui| {
            ui.set_width(width - 20.0);
            add(ui)
        })
        .inner;
    ui.add_space(8.0);
    Some(r)
}

/// A label + widget row inside a task box (label left, editor right-aligned at 45%).
pub fn task_row<R>(ui: &mut egui::Ui, label: &str, add: impl FnOnce(&mut egui::Ui) -> R) -> R {
    let w = ui.available_width();
    ui.horizontal(|ui| {
        ui.set_min_height(26.0);
        let lw = (w * 0.42).max(80.0);
        let (r, _) = ui.allocate_exact_size(egui::vec2(lw, 26.0), Sense::hover());
        ui.painter().text(Pos2::new(r.left(), r.center().y), Align2::LEFT_CENTER, label, egui::FontId::proportional(13.0), theme::TEXT);
        add(ui)
    })
    .inner
}

// ---------------------------------------------------------------- property grid

pub struct PropGrid {
    pub name_w: f32,
    row: usize,
}

impl PropGrid {
    pub fn new(ui: &egui::Ui) -> Self {
        PropGrid { name_w: (ui.available_width() * 0.45).clamp(90.0, 190.0), row: 0 }
    }

    /// Bold group header row ("Base", "Pad").
    pub fn group(&mut self, ui: &mut egui::Ui, title: &str) {
        let (rect, _) = ui.allocate_exact_size(egui::vec2(ui.available_width(), theme::ROW_H), Sense::hover());
        ui.painter().rect_filled(rect, 0.0, theme::GROUP_ROW);
        ui.painter().text(Pos2::new(rect.left() + 5.0, rect.center().y), Align2::LEFT_CENTER, title, theme::bold_font(13.0), theme::TEXT);
        self.row = 0;
    }

    /// Property row: name cell on the left, `value` draws the editor in the right cell.
    pub fn row<R>(&mut self, ui: &mut egui::Ui, name: &str, tip: &str, value: impl FnOnce(&mut egui::Ui) -> R) -> R {
        let name_s = name.to_string();
        let tip_s = tip.to_string();
        self.row_with(
            ui,
            &format!("prop:{name}"),
            |ui| {
                let w = ui.available_width();
                let galley = ui.painter().layout_no_wrap(name_s.clone(), egui::FontId::proportional(13.0), theme::TEXT);
                let shown = if galley.size().x > w { elide(ui, &name_s, w) } else { name_s.clone() };
                let r = ui.add(egui::Label::new(RichText::new(&shown).color(theme::TEXT)).selectable(false).truncate());
                if !tip_s.is_empty() || shown != name_s {
                    r.on_hover_ui(|ui| tooltip(ui, &name_s, &tip_s, ""));
                }
            },
            value,
        )
    }

    /// Row with custom widgets in both cells; `reg_name` names the value cell for scripts.
    pub fn row_with<R>(&mut self, ui: &mut egui::Ui, reg_name: &str, name: impl FnOnce(&mut egui::Ui), value: impl FnOnce(&mut egui::Ui) -> R) -> R {
        let w = ui.available_width();
        let top = ui.cursor().top();
        let rect = Rect::from_min_size(Pos2::new(ui.cursor().left(), top), egui::vec2(w, theme::ROW_H));
        let bg = if self.row % 2 == 0 { theme::ROW } else { theme::ROW_ALT };
        self.row += 1;
        ui.painter().rect_filled(rect, 0.0, bg);
        let split = rect.left() + self.name_w;
        ui.painter().line_segment([Pos2::new(split, rect.top()), Pos2::new(split, rect.bottom())], Stroke::new(1.0, theme::GRID_LINE));
        ui.painter().line_segment([rect.left_bottom(), rect.right_bottom()], Stroke::new(1.0, Color32::from_rgb(30, 30, 30)));
        let name_rect = Rect::from_min_max(rect.min + egui::vec2(16.0, 1.0), Pos2::new(split - 4.0, rect.bottom() - 1.0));
        ui.scope_builder(egui::UiBuilder::new().max_rect(name_rect).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
            ui.set_clip_rect(name_rect.intersect(ui.clip_rect()));
            ui.spacing_mut().interact_size.y = theme::ROW_H - 2.0;
            name(ui)
        });
        let value_rect = Rect::from_min_max(Pos2::new(split + 4.0, rect.top() + 1.0), Pos2::new(rect.right() - 2.0, rect.bottom() - 1.0));
        reg(reg_name.to_string(), value_rect);
        let r = ui.scope_builder(egui::UiBuilder::new().max_rect(value_rect).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
            ui.spacing_mut().interact_size.y = theme::ROW_H - 2.0;
            ui.spacing_mut().button_padding = egui::vec2(4.0, 1.0);
            ui.spacing_mut().item_spacing.x = 4.0;
            value(ui)
        });
        ui.advance_cursor_after_rect(rect);
        r.inner
    }
}

/// Shorten `s` with an ellipsis to fit `w` points (Qt's elided item text).
pub fn elide(ui: &egui::Ui, s: &str, w: f32) -> String {
    let font = egui::FontId::proportional(13.0);
    let mut chars: Vec<char> = s.chars().collect();
    while !chars.is_empty() {
        let t: String = chars.iter().collect::<String>() + "…";
        if ui.painter().layout_no_wrap(t.clone(), font.clone(), Color32::WHITE).size().x <= w {
            return t;
        }
        chars.pop();
    }
    "…".into()
}

/// Read-only value text in a property cell (gray like FreeCAD's read-only properties).
pub fn ro_value(ui: &mut egui::Ui, text: &str) {
    ui.add(egui::Label::new(RichText::new(text).color(theme::TEXT_DIM)).truncate());
}

/// Bool property: checkbox + Yes/No like FreeCAD.
pub fn yes_no(ui: &mut egui::Ui, v: &mut bool) -> bool {
    ui.checkbox(v, "").on_hover_text(if *v { "Yes" } else { "No" }).changed()
}

/// A dropdown whose options register as `opt:<id>:<label>` for the input script.
pub fn combo<T: PartialEq + Clone>(ui: &mut egui::Ui, id: &str, current: &mut T, options: &[(T, &str)], width: f32) -> bool {
    let sel = options.iter().find(|(v, _)| v == current).map(|(_, l)| *l).unwrap_or("");
    let mut changed = false;
    let r = egui::ComboBox::from_id_salt(id).selected_text(sel).width(width).show_ui(ui, |ui| {
        for (v, l) in options {
            let r = ui.selectable_label(v == current, *l);
            reg(format!("opt:{id}:{l}"), r.rect);
            if r.clicked() && v != current {
                *current = v.clone();
                changed = true;
            }
        }
    });
    reg(format!("combo:{id}"), r.response.rect);
    changed
}

/// Tab strip (Qt's QTabBar at the bottom of the property view). Returns the clicked tab.
pub fn tab_bar(ui: &mut egui::Ui, tabs: &[&str], current: usize) -> Option<usize> {
    let mut out = None;
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        let total: f32 = tabs.iter().map(|t| ui.painter().layout_no_wrap(t.to_string(), egui::FontId::proportional(13.0), Color32::WHITE).size().x + 20.0).sum();
        ui.add_space(((ui.available_width() - total) * 0.5).max(0.0));
        for (i, t) in tabs.iter().enumerate() {
            let w = ui.painter().layout_no_wrap(t.to_string(), egui::FontId::proportional(13.0), Color32::WHITE).size().x + 20.0;
            let (rect, resp) = ui.allocate_exact_size(egui::vec2(w, 24.0), Sense::click());
            reg(format!("tab:{t}"), rect);
            let sel = i == current;
            let fill = if sel { Color32::from_rgb(48, 48, 48) } else if resp.hovered() { Color32::from_rgb(36, 36, 36) } else { theme::PANEL };
            let cr = CornerRadius { nw: 0, ne: 0, sw: 4, se: 4 };
            ui.painter().rect_filled(rect, cr, fill);
            if sel {
                ui.painter().rect_stroke(rect, cr, Stroke::new(1.0, theme::BORDER), egui::StrokeKind::Inside);
                ui.painter().line_segment([rect.left_bottom() + egui::vec2(4.0, -1.0), rect.right_bottom() + egui::vec2(-4.0, -1.0)], Stroke::new(2.0, theme::ACCENT));
            }
            let font = if sel { theme::bold_font(13.0) } else { egui::FontId::proportional(13.0) };
            ui.painter().text(rect.center(), Align2::CENTER_CENTER, *t, font, if sel { Color32::WHITE } else { theme::TEXT_DIM });
            if resp.clicked() {
                out = Some(i);
            }
        }
    });
    out
}
