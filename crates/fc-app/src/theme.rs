//! FreeCAD 1.1 dark theme: palette, fonts, compact metrics.

use eframe::egui::{self, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, FontId, Margin, Stroke, TextStyle};

pub const WINDOW: Color32 = Color32::from_rgb(25, 25, 25);
pub const PANEL: Color32 = Color32::from_rgb(25, 25, 25);
/// Property grid rows, task box bodies.
pub const ROW: Color32 = Color32::from_rgb(35, 35, 35);
pub const ROW_ALT: Color32 = Color32::from_rgb(40, 40, 40);
pub const GROUP_ROW: Color32 = Color32::from_rgb(52, 52, 52);
pub const BOX_BODY: Color32 = Color32::from_rgb(45, 45, 45);
pub const BOX_HEADER: Color32 = Color32::from_rgb(38, 38, 38);
pub const INPUT: Color32 = Color32::from_rgb(25, 25, 25);
pub const BUTTON: Color32 = Color32::from_rgb(44, 44, 44);
pub const BUTTON_HOVER: Color32 = Color32::from_rgb(58, 58, 58);
pub const BORDER: Color32 = Color32::from_rgb(62, 62, 62);
pub const GRID_LINE: Color32 = Color32::from_rgb(52, 52, 52);
pub const TEXT: Color32 = Color32::from_rgb(220, 220, 220);
pub const TEXT_DIM: Color32 = Color32::from_rgb(140, 140, 140);
pub const SELECTION: Color32 = Color32::from_rgb(54, 109, 163);
pub const HOVER_ROW: Color32 = Color32::from_rgb(48, 58, 70);
pub const ACCENT: Color32 = Color32::from_rgb(40, 125, 200);
pub const OK_BUTTON: Color32 = Color32::from_rgb(33, 54, 77);
pub const GREEN: Color32 = Color32::from_rgb(140, 255, 91);
pub const ERROR: Color32 = Color32::from_rgb(240, 90, 80);
pub const WARNING: Color32 = Color32::from_rgb(235, 175, 50);
/// The 3D view background (FreeCAD 1.1 with the dark theme, measured with saveImage).
pub const VIEW_BG: [f32; 3] = [31.0 / 255.0, 31.0 / 255.0, 31.0 / 255.0];

pub const FONT_SIZE: f32 = 13.0;
pub const ROW_H: f32 = 22.0;

pub fn bold() -> FontFamily {
    FontFamily::Name("bold".into())
}

pub fn bold_font(size: f32) -> FontId {
    FontId::new(size, bold())
}

/// The macOS system font (SF Pro, as Qt uses) when available, else egui's default.
fn install_fonts(ctx: &egui::Context) {
    let mut fonts = FontDefinitions::default();
    let sys = ["/System/Library/Fonts/SFNS.ttf", "/System/Library/Fonts/Supplemental/Arial.ttf"].iter().find_map(|p| std::fs::read(p).ok());
    let mut bold_name = None;
    if let Some(data) = sys {
        let regular = FontData::from_owned(data.clone());
        let is_var = !regular.variation_axes().is_empty();
        fonts.font_data.insert("system".into(), regular.into());
        fonts.families.get_mut(&FontFamily::Proportional).unwrap().insert(0, "system".into());
        if is_var {
            let mut tweak = egui::epaint::text::FontTweak::default();
            tweak.coords = egui::epaint::text::VariationCoords::new([("wght", 650.0)]);
            fonts.font_data.insert("system-bold".into(), FontData::from_owned(data).tweak(tweak).into());
            bold_name = Some("system-bold");
        } else if let Ok(b) = std::fs::read("/System/Library/Fonts/Supplemental/Arial Bold.ttf") {
            fonts.font_data.insert("system-bold".into(), FontData::from_owned(b).into());
            bold_name = Some("system-bold");
        }
    }
    let mut bold_family: Vec<String> = bold_name.map(|n| vec![n.to_string()]).unwrap_or_default();
    bold_family.extend(fonts.families[&FontFamily::Proportional].iter().cloned());
    fonts.families.insert(bold(), bold_family);
    ctx.set_fonts(fonts);
}

pub fn apply(ctx: &egui::Context) {
    install_fonts(ctx);
    ctx.set_theme(egui::Theme::Dark);
    ctx.global_style_mut(|style| {
        style.text_styles = [
            (TextStyle::Small, FontId::proportional(11.0)),
            (TextStyle::Body, FontId::proportional(FONT_SIZE)),
            (TextStyle::Button, FontId::proportional(FONT_SIZE)),
            (TextStyle::Heading, FontId::new(15.0, bold())),
            (TextStyle::Monospace, FontId::monospace(12.0)),
        ]
        .into();
        let s = &mut style.spacing;
        s.item_spacing = egui::vec2(6.0, 4.0);
        s.button_padding = egui::vec2(8.0, 3.0);
        s.interact_size = egui::vec2(24.0, 22.0);
        s.menu_margin = Margin::same(4);
        s.window_margin = Margin::same(10);
        s.combo_height = 300.0;
        s.icon_width = 14.0;
        s.icon_width_inner = 8.0;
        s.scroll.bar_width = 8.0;
        s.scroll.floating = true;
        let v = &mut style.visuals;
        v.dark_mode = true;
        v.override_text_color = None;
        v.panel_fill = PANEL;
        v.window_fill = Color32::from_rgb(36, 36, 36);
        v.window_stroke = Stroke::new(1.0, BORDER);
        v.window_corner_radius = CornerRadius::same(6);
        v.menu_corner_radius = CornerRadius::same(5);
        v.extreme_bg_color = INPUT;
        v.faint_bg_color = ROW_ALT;
        v.code_bg_color = ROW;
        v.selection.bg_fill = SELECTION;
        v.selection.stroke = Stroke::new(1.0, Color32::WHITE);
        v.hyperlink_color = Color32::from_rgb(110, 170, 255);
        v.warn_fg_color = WARNING;
        v.error_fg_color = ERROR;
        v.striped = false;
        v.slider_trailing_fill = true;
        v.handle_shape = egui::style::HandleShape::Circle;
        v.text_cursor.stroke = Stroke::new(1.5, Color32::WHITE);
        let w = &mut v.widgets;
        let r = CornerRadius::same(4);
        w.noninteractive.bg_fill = PANEL;
        w.noninteractive.weak_bg_fill = PANEL;
        w.noninteractive.bg_stroke = Stroke::new(1.0, GRID_LINE);
        w.noninteractive.fg_stroke = Stroke::new(1.0, TEXT);
        w.noninteractive.corner_radius = r;
        w.inactive.bg_fill = BUTTON;
        w.inactive.weak_bg_fill = BUTTON;
        w.inactive.bg_stroke = Stroke::new(1.0, BORDER);
        w.inactive.fg_stroke = Stroke::new(1.0, TEXT);
        w.inactive.corner_radius = r;
        w.hovered.bg_fill = BUTTON_HOVER;
        w.hovered.weak_bg_fill = BUTTON_HOVER;
        w.hovered.bg_stroke = Stroke::new(1.0, Color32::from_rgb(90, 90, 90));
        w.hovered.fg_stroke = Stroke::new(1.0, Color32::WHITE);
        w.hovered.corner_radius = r;
        w.hovered.expansion = 0.0;
        w.active.bg_fill = Color32::from_rgb(70, 70, 70);
        w.active.weak_bg_fill = Color32::from_rgb(70, 70, 70);
        w.active.bg_stroke = Stroke::new(1.0, ACCENT);
        w.active.fg_stroke = Stroke::new(1.0, Color32::WHITE);
        w.active.corner_radius = r;
        w.active.expansion = 0.0;
        w.open.bg_fill = Color32::from_rgb(50, 50, 50);
        w.open.weak_bg_fill = Color32::from_rgb(50, 50, 50);
        w.open.bg_stroke = Stroke::new(1.0, ACCENT);
        w.open.corner_radius = r;
        style.interaction.tooltip_delay = 0.45;
        style.interaction.show_tooltips_only_when_still = true;
    });
}
