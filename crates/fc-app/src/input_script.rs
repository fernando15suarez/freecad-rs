//! `FCRS_INPUT=script.txt`: drives the app through egui's raw input (the same path real
//! mouse and keyboard events take), one step per frame. Used to test the UI end to end.
//!
//! Lines (`#` comments):
//!   at sketch X Y        move pointer to sketch coordinates of the sketch being edited
//!   at world X Y Z       move pointer to the screen projection of a world point
//!   at screen X Y        move pointer to logical screen coordinates
//!   down / up [right|middle]   press / release a mouse button at the pointer
//!   click [right]        press + release
//!   dclick               double click
//!   key NAME [cmd|shift|ctrl|alt ...]   key press + release (NAME: egui key name, e.g. Escape, G, Delete)
//!   type TEXT            text input (into the focused text field)
//!   scroll DX DY         scroll wheel
//!   wait N               idle N frames
//!   expect dof N | expect editing true|false | expect features N | expect volume V
//!   demo | saveto PATH | external NAME EXPR   (test helpers: load the demo, save to PATH, and rewrite the open
//!     file on disk the way Calc Canvas does: parameter NAME = EXPR, marked driven)
//!   expect measure NAME V | param NAME V | driven NAME | banner true|false | dirty true|false
//!   shot PATH            save a screenshot
//!   quit

use eframe::egui::{self, Event, Modifiers, PointerButton, Pos2};

#[derive(Debug, Clone)]
pub enum Step {
    AtSketch(f64, f64),
    AtWorld([f64; 3]),
    AtScreen(f32, f32),
    Down(PointerButton),
    Up(PointerButton),
    Click(PointerButton),
    DoubleClick,
    Key(egui::Key, Modifiers),
    Type(String),
    Scroll(f32, f32),
    Wait(u32),
    Expect(String, String),
    Shot(String),
    Do(String, String),
    Quit,
}

pub fn parse(src: &str) -> Result<Vec<Step>, String> {
    let mut out = Vec::new();
    for (ln, line) in src.lines().enumerate() {
        let line = line.split('#').next().unwrap().trim();
        if line.is_empty() {
            continue;
        }
        let w: Vec<&str> = line.split_whitespace().collect();
        let f = |i: usize| -> Result<f64, String> { w.get(i).and_then(|s| s.parse().ok()).ok_or(format!("line {}: bad number", ln + 1)) };
        let button = |i: usize| match w.get(i).copied() {
            Some("right") => PointerButton::Secondary,
            Some("middle") => PointerButton::Middle,
            _ => PointerButton::Primary,
        };
        let step = match w[0] {
            "at" => match w.get(1).copied() {
                Some("sketch") => Step::AtSketch(f(2)?, f(3)?),
                Some("world") => Step::AtWorld([f(2)?, f(3)?, f(4)?]),
                Some("screen") => Step::AtScreen(f(2)? as f32, f(3)? as f32),
                _ => return Err(format!("line {}: at sketch|world|screen", ln + 1)),
            },
            "down" => Step::Down(button(1)),
            "up" => Step::Up(button(1)),
            "click" => Step::Click(button(1)),
            "dclick" => Step::DoubleClick,
            "key" => {
                let key = egui::Key::from_name(w.get(1).copied().unwrap_or("")).ok_or(format!("line {}: unknown key", ln + 1))?;
                let mut m = Modifiers::default();
                for x in &w[2..] {
                    match *x {
                        "cmd" => {
                            m.command = true;
                            m.mac_cmd = true;
                        }
                        "shift" => m.shift = true,
                        "ctrl" => m.ctrl = true,
                        "alt" => m.alt = true,
                        _ => {}
                    }
                }
                Step::Key(key, m)
            }
            "type" => Step::Type(line[5..].to_string()),
            "scroll" => Step::Scroll(f(1)? as f32, f(2)? as f32),
            "wait" => Step::Wait(f(1)? as u32),
            "expect" => Step::Expect(w.get(1).unwrap_or(&"").to_string(), w[2.min(w.len())..].join(" ")),
            "demo" | "saveto" | "external" => Step::Do(w[0].to_string(), line[w[0].len()..].trim().to_string()),
            "shot" => Step::Shot(w.get(1).unwrap_or(&"").to_string()),
            "quit" => Step::Quit,
            other => return Err(format!("line {}: unknown step '{other}'", ln + 1)),
        };
        out.push(step);
    }
    Ok(out)
}

/// Runner state: pending steps and the last pointer position.
pub struct Runner {
    pub steps: std::collections::VecDeque<Step>,
    pub pointer: Pos2,
    pub wait: u32,
    pub failures: u32,
    /// A click in progress: release on the next frame.
    pub pending: Vec<Event>,
}

impl Runner {
    pub fn new(steps: Vec<Step>) -> Self {
        Runner { steps: steps.into(), pointer: Pos2::new(10.0, 10.0), wait: 0, failures: 0, pending: vec![] }
    }

    pub fn button_event(&self, button: PointerButton, pressed: bool) -> Event {
        Event::PointerButton { pos: self.pointer, button, pressed, modifiers: Modifiers::default() }
    }
}
