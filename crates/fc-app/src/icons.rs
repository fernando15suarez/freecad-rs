//! FreeCAD's SVG icons (assets/icons, LGPL-2.1, see NOTICE), rasterized on demand at the
//! display's pixel density and cached as egui textures.

use eframe::egui::{self, Color32, ColorImage, TextureHandle, TextureOptions};
use std::cell::RefCell;
use std::collections::HashMap;

include!(concat!(env!("OUT_DIR"), "/icons.rs"));

thread_local! {
    static CACHE: RefCell<HashMap<(&'static str, u32, bool), Option<TextureHandle>>> = RefCell::new(HashMap::new());
}

fn svg_bytes(name: &str) -> Option<(&'static str, &'static [u8])> {
    ICONS.iter().find(|(n, _)| *n == name).map(|(n, b)| (*n, *b))
}

fn rasterize(data: &[u8], px: u32) -> Option<ColorImage> {
    let tree = resvg::usvg::Tree::from_data(data, &resvg::usvg::Options::default()).ok()?;
    let mut pm = resvg::tiny_skia::Pixmap::new(px, px)?;
    let s = tree.size();
    let scale = px as f32 / s.width().max(s.height());
    let (dx, dy) = ((px as f32 - s.width() * scale) * 0.5, (px as f32 - s.height() * scale) * 0.5);
    resvg::render(&tree, resvg::tiny_skia::Transform::from_row(scale, 0.0, 0.0, scale, dx, dy), &mut pm.as_mut());
    // tiny-skia stores premultiplied RGBA, which is what egui's Color32 is.
    let pixels = pm.pixels().iter().map(|p| Color32::from_rgba_premultiplied(p.red(), p.green(), p.blue(), p.alpha())).collect();
    Some(ColorImage { size: [px as usize, px as usize], pixels, source_size: egui::vec2(px as f32, px as f32) })
}

/// Texture of icon `name` (file stem in assets/icons) for drawing at `size` points.
pub fn texture(ctx: &egui::Context, name: &str, size: f32) -> Option<egui::TextureId> {
    texture_ex(ctx, name, size, false)
}

/// Like [`texture`]; `disabled` gives the faded grayscale version Qt shows for disabled actions.
pub fn texture_ex(ctx: &egui::Context, name: &str, size: f32, disabled: bool) -> Option<egui::TextureId> {
    let (name, data) = svg_bytes(name)?;
    let px = (size * ctx.pixels_per_point()).round().max(1.0) as u32;
    CACHE.with(|c| {
        let mut c = c.borrow_mut();
        c.entry((name, px, disabled))
            .or_insert_with(|| {
                rasterize(data, px).map(|mut img| {
                    if disabled {
                        for p in &mut img.pixels {
                            let [r, g, b, a] = p.to_array();
                            // un-premultiply, gray it out, lighten, then fade
                            let un = |v: u8| if a == 0 { 0.0 } else { v as f32 * 255.0 / a as f32 };
                            let l = (un(r) * 0.3 + un(g) * 0.6 + un(b) * 0.1) * 0.45 + 70.0;
                            let na = a as f32 * 0.55;
                            let v = (l.min(255.0) * na / 255.0) as u8;
                            *p = Color32::from_rgba_premultiplied(v, v, v, na as u8);
                        }
                    }
                    ctx.load_texture(format!("icon:{name}:{px}:{disabled}"), img, TextureOptions::LINEAR)
                })
            })
            .as_ref()
            .map(|t| t.id())
    })
}

/// An icon as an `egui::Image` of `size` points (empty image if the icon is missing).
pub fn image(ctx: &egui::Context, name: &str, size: f32) -> egui::Image<'static> {
    match texture(ctx, name, size) {
        Some(id) => egui::Image::new(egui::load::SizedTexture::new(id, egui::vec2(size, size))),
        None => egui::Image::new(egui::load::SizedTexture::new(egui::TextureId::default(), egui::vec2(size, size))).tint(Color32::TRANSPARENT),
    }
}

/// Paint icon `name` into `rect` (square), optionally tinted (e.g. gray for disabled).
pub fn paint(painter: &egui::Painter, name: &str, rect: egui::Rect, tint: Color32) {
    paint_ex(painter, name, rect, tint, false)
}

pub fn paint_ex(painter: &egui::Painter, name: &str, rect: egui::Rect, tint: Color32, disabled: bool) {
    if let Some(id) = texture_ex(painter.ctx(), name, rect.width(), disabled) {
        painter.image(id, rect, egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0)), tint);
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn all_icons_rasterize() {
        for (name, data) in super::ICONS {
            let img = super::rasterize(data, 32).unwrap_or_else(|| panic!("{name} failed to rasterize"));
            assert!(img.pixels.iter().any(|p| p.a() > 0), "{name} is empty");
        }
    }
}
