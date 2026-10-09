//! Named dimensions and expressions, after FreeCAD's ExpressionEngine.
//!
//! Syntax: numbers with optional units (`mm cm m in ft deg rad °`), `+ - * / ^ %`,
//! parentheses, functions (`sin cos tan asin acos atan atan2 sqrt abs min max floor ceil
//! round pow exp log log10 hypot`), constants `pi`/`e`, and references:
//! - `width` — a document parameter, or a named sketch constraint anywhere in the body
//! - `Sketch.width` / `Sketch.Constraints.width` / `Sketch.Constraints[3]`
//! - `Pad.Length`, `Revolution.Angle`, `Hole.Diameter`, ... (see [`feature_props`])
//!
//! Angles are in degrees inside expressions, like FreeCAD's UI; sketch angle constraints
//! (stored in radians) are converted on read and write.

use std::collections::{BTreeMap, HashSet};
use std::fmt;

#[derive(Clone, Debug, PartialEq)]
pub enum Expr {
    Num(f64),
    Ref(String),
    Neg(Box<Expr>),
    Bin(char, Box<Expr>, Box<Expr>),
    Call(String, Vec<Expr>),
}

#[derive(Clone, Debug, PartialEq)]
pub struct ExprError(pub String);
impl fmt::Display for ExprError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for ExprError {}

fn err<T>(s: impl Into<String>) -> Result<T, ExprError> {
    Err(ExprError(s.into()))
}

struct Parser<'a> {
    s: &'a [u8],
    i: usize,
}

impl<'a> Parser<'a> {
    fn ws(&mut self) {
        while self.i < self.s.len() && (self.s[self.i] as char).is_whitespace() {
            self.i += 1;
        }
    }
    fn peek(&mut self) -> Option<char> {
        self.ws();
        self.s.get(self.i).map(|&c| c as char)
    }
    fn eat(&mut self, c: char) -> bool {
        if self.peek() == Some(c) {
            self.i += 1;
            true
        } else {
            false
        }
    }
    fn expr(&mut self) -> Result<Expr, ExprError> {
        let mut l = self.term()?;
        loop {
            match self.peek() {
                Some(c @ ('+' | '-')) => {
                    self.i += 1;
                    let r = self.term()?;
                    l = Expr::Bin(c, Box::new(l), Box::new(r));
                }
                _ => return Ok(l),
            }
        }
    }
    fn term(&mut self) -> Result<Expr, ExprError> {
        let mut l = self.unary()?;
        loop {
            match self.peek() {
                Some(c @ ('*' | '/' | '%')) => {
                    self.i += 1;
                    let r = self.unary()?;
                    l = Expr::Bin(c, Box::new(l), Box::new(r));
                }
                _ => return Ok(l),
            }
        }
    }
    fn unary(&mut self) -> Result<Expr, ExprError> {
        if self.eat('-') {
            return Ok(Expr::Neg(Box::new(self.unary()?)));
        }
        if self.eat('+') {
            return self.unary();
        }
        self.power()
    }
    fn power(&mut self) -> Result<Expr, ExprError> {
        let b = self.atom()?;
        if self.eat('^') {
            let e = self.unary()?; // right associative
            return Ok(Expr::Bin('^', Box::new(b), Box::new(e)));
        }
        Ok(b)
    }
    fn ident(&mut self) -> String {
        let st = self.i;
        while self.i < self.s.len() {
            let c = self.s[self.i] as char;
            if c.is_ascii_alphanumeric() || c == '_' {
                self.i += 1;
            } else {
                break;
            }
        }
        String::from_utf8_lossy(&self.s[st..self.i]).into_owned()
    }
    fn unit(&mut self) -> f64 {
        self.ws();
        let rest = &self.s[self.i..];
        if rest.starts_with("°".as_bytes()) {
            self.i += "°".len();
            return 1.0;
        }
        let save = self.i;
        let id = self.ident();
        let f = match id.as_str() {
            "mm" => 1.0,
            "cm" => 10.0,
            "m" => 1000.0,
            "um" => 1e-3,
            "in" => 25.4,
            "ft" => 304.8,
            "deg" => 1.0,
            "rad" => 180.0 / std::f64::consts::PI,
            _ => {
                self.i = save;
                1.0
            }
        };
        // A unit must not be followed by '.', '(' or '[' (then it was an identifier).
        if f != 1.0 || id == "mm" || id == "deg" {
            if matches!(self.s.get(self.i).map(|&c| c as char), Some('.' | '(' | '[')) {
                self.i = save;
                return 1.0;
            }
        }
        f
    }
    fn atom(&mut self) -> Result<Expr, ExprError> {
        match self.peek() {
            Some('(') => {
                self.i += 1;
                let e = self.expr()?;
                if !self.eat(')') {
                    return err("missing ')'");
                }
                Ok(e)
            }
            Some(c) if c.is_ascii_digit() || c == '.' => {
                let st = self.i;
                while self.i < self.s.len() && ((self.s[self.i] as char).is_ascii_digit() || self.s[self.i] == b'.') {
                    self.i += 1;
                }
                if self.i < self.s.len() && (self.s[self.i] == b'e' || self.s[self.i] == b'E') {
                    let save = self.i;
                    self.i += 1;
                    if self.i < self.s.len() && (self.s[self.i] == b'-' || self.s[self.i] == b'+') {
                        self.i += 1;
                    }
                    if self.i < self.s.len() && (self.s[self.i] as char).is_ascii_digit() {
                        while self.i < self.s.len() && (self.s[self.i] as char).is_ascii_digit() {
                            self.i += 1;
                        }
                    } else {
                        self.i = save;
                    }
                }
                let txt = std::str::from_utf8(&self.s[st..self.i]).unwrap();
                let v: f64 = txt.parse().map_err(|_| ExprError(format!("bad number '{txt}'")))?;
                Ok(Expr::Num(v * self.unit()))
            }
            Some('<') => {
                // FreeCAD label syntax <<My Label>>
                if !(self.eat('<') && self.eat('<')) {
                    return err("expected '<<'");
                }
                let st = self.i;
                while self.i + 1 < self.s.len() && !(self.s[self.i] == b'>' && self.s[self.i + 1] == b'>') {
                    self.i += 1;
                }
                let label = String::from_utf8_lossy(&self.s[st..self.i]).into_owned();
                self.i += 2;
                let mut path = label;
                self.path_tail(&mut path)?;
                Ok(Expr::Ref(path))
            }
            Some(c) if c.is_ascii_alphabetic() || c == '_' => {
                let mut id = self.ident();
                if self.peek() == Some('(') {
                    self.i += 1;
                    let mut args = Vec::new();
                    if !self.eat(')') {
                        loop {
                            args.push(self.expr()?);
                            if self.eat(')') {
                                break;
                            }
                            if !self.eat(',') && !self.eat(';') {
                                return err("expected ',' or ')'");
                            }
                        }
                    }
                    return Ok(Expr::Call(id, args));
                }
                self.path_tail(&mut id)?;
                Ok(Expr::Ref(id))
            }
            Some(c) => err(format!("unexpected '{c}'")),
            None => err("unexpected end of expression"),
        }
    }
    fn path_tail(&mut self, path: &mut String) -> Result<(), ExprError> {
        loop {
            if self.s.get(self.i) == Some(&b'.') {
                self.i += 1;
                let id = self.ident();
                if id.is_empty() {
                    return err("expected name after '.'");
                }
                path.push('.');
                path.push_str(&id);
            } else if self.s.get(self.i) == Some(&b'[') {
                self.i += 1;
                let st = self.i;
                while self.i < self.s.len() && self.s[self.i] != b']' {
                    self.i += 1;
                }
                let inner = String::from_utf8_lossy(&self.s[st..self.i]).trim().to_string();
                self.i += 1;
                path.push('[');
                path.push_str(&inner);
                path.push(']');
            } else {
                return Ok(());
            }
        }
    }
}

pub fn parse(src: &str) -> Result<Expr, ExprError> {
    let src = src.trim();
    let src = src.strip_prefix('=').unwrap_or(src);
    let mut p = Parser { s: src.as_bytes(), i: 0 };
    let e = p.expr()?;
    if p.peek().is_some() {
        return err(format!("unexpected '{}'", &src[p.i..]));
    }
    Ok(e)
}

impl Expr {
    pub fn refs(&self, out: &mut Vec<String>) {
        match self {
            Expr::Num(_) => {}
            Expr::Ref(r) => out.push(r.clone()),
            Expr::Neg(e) => e.refs(out),
            Expr::Bin(_, a, b) => {
                a.refs(out);
                b.refs(out);
            }
            Expr::Call(_, a) => a.iter().for_each(|e| e.refs(out)),
        }
    }

    pub fn eval(&self, lookup: &mut dyn FnMut(&str) -> Result<f64, ExprError>) -> Result<f64, ExprError> {
        Ok(match self {
            Expr::Num(v) => *v,
            Expr::Ref(r) => match r.as_str() {
                "pi" => std::f64::consts::PI,
                "e" => std::f64::consts::E,
                _ => lookup(r)?,
            },
            Expr::Neg(e) => -e.eval(lookup)?,
            Expr::Bin(op, a, b) => {
                let (a, b) = (a.eval(lookup)?, b.eval(lookup)?);
                match op {
                    '+' => a + b,
                    '-' => a - b,
                    '*' => a * b,
                    '/' => {
                        if b == 0.0 {
                            return err("division by zero");
                        }
                        a / b
                    }
                    '%' => a % b,
                    '^' => a.powf(b),
                    _ => unreachable!(),
                }
            }
            Expr::Call(f, args) => {
                let v: Vec<f64> = args.iter().map(|a| a.eval(lookup)).collect::<Result<_, _>>()?;
                let n = v.len();
                let need = |k: usize| if n == k { Ok(()) } else { err(format!("{f}() takes {k} argument(s)")) };
                let d2r = std::f64::consts::PI / 180.0;
                match f.as_str() {
                    "sin" => { need(1)?; (v[0] * d2r).sin() }
                    "cos" => { need(1)?; (v[0] * d2r).cos() }
                    "tan" => { need(1)?; (v[0] * d2r).tan() }
                    "asin" => { need(1)?; v[0].asin() / d2r }
                    "acos" => { need(1)?; v[0].acos() / d2r }
                    "atan" => { need(1)?; v[0].atan() / d2r }
                    "atan2" => { need(2)?; v[0].atan2(v[1]) / d2r }
                    "sqrt" => { need(1)?; v[0].sqrt() }
                    "abs" => { need(1)?; v[0].abs() }
                    "floor" => { need(1)?; v[0].floor() }
                    "ceil" => { need(1)?; v[0].ceil() }
                    "round" => { need(1)?; v[0].round() }
                    "exp" => { need(1)?; v[0].exp() }
                    "log" => { need(1)?; v[0].ln() }
                    "log10" => { need(1)?; v[0].log10() }
                    "pow" => { need(2)?; v[0].powf(v[1]) }
                    "hypot" => { need(2)?; v[0].hypot(v[1]) }
                    "min" if n > 0 => v.iter().cloned().fold(f64::INFINITY, f64::min),
                    "max" if n > 0 => v.iter().cloned().fold(f64::NEG_INFINITY, f64::max),
                    _ => return err(format!("unknown function {f}()")),
                }
            }
        })
    }
}

/// Evaluate a set of named expressions that may reference each other and external values.
/// Returns values in the order they can be computed; reports cycles.
pub fn eval_all(
    exprs: &BTreeMap<String, Expr>,
    external: &mut dyn FnMut(&str) -> Option<f64>,
    canonical: &dyn Fn(&str) -> Option<String>,
) -> (BTreeMap<String, f64>, BTreeMap<String, ExprError>) {
    let mut done: BTreeMap<String, f64> = BTreeMap::new();
    let mut errors: BTreeMap<String, ExprError> = BTreeMap::new();
    fn visit(
        key: &str,
        exprs: &BTreeMap<String, Expr>,
        external: &mut dyn FnMut(&str) -> Option<f64>,
        canonical: &dyn Fn(&str) -> Option<String>,
        done: &mut BTreeMap<String, f64>,
        errors: &mut BTreeMap<String, ExprError>,
        stack: &mut HashSet<String>,
    ) -> Result<f64, ExprError> {
        if let Some(v) = done.get(key) {
            return Ok(*v);
        }
        if let Some(e) = errors.get(key) {
            return Err(e.clone());
        }
        let Some(ex) = exprs.get(key) else {
            return external(key).ok_or_else(|| ExprError(format!("unknown name '{key}'")));
        };
        if !stack.insert(key.to_string()) {
            return err(format!("circular reference through '{key}'"));
        }
        let ex = ex.clone();
        let r = ex.eval(&mut |name: &str| {
            let k = canonical(name).ok_or_else(|| ExprError(format!("unknown name '{name}'")))?;
            visit(&k, exprs, external, canonical, done, errors, stack)
        });
        stack.remove(key);
        match &r {
            Ok(v) => {
                done.insert(key.to_string(), *v);
            }
            Err(e) => {
                errors.insert(key.to_string(), e.clone());
            }
        }
        r
    }
    for k in exprs.keys() {
        let mut stack = HashSet::new();
        let _ = visit(k, exprs, external, canonical, &mut done, &mut errors, &mut stack);
    }
    (done, errors)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ev(s: &str) -> f64 {
        parse(s).unwrap().eval(&mut |n| match n {
            "width" => Ok(20.0),
            "Pad.Length" => Ok(5.0),
            "Sketch.Constraints[3]" => Ok(7.0),
            "My Sketch.w" => Ok(2.0),
            _ => err(format!("unknown {n}")),
        }).unwrap()
    }

    #[test]
    fn arithmetic_units_functions() {
        assert_eq!(ev("1 + 2 * 3"), 7.0);
        assert_eq!(ev("(1 + 2) * 3"), 9.0);
        assert_eq!(ev("-2 ^ 2"), -4.0);
        assert_eq!(ev("2 ^ 3 ^ 2"), 512.0);
        assert_eq!(ev("= 1 cm + 5 mm"), 15.0);
        assert_eq!(ev("1in"), 25.4);
        assert!((ev("sin(30)") - 0.5).abs() < 1e-12);
        assert!((ev("1 rad") - 57.29577951308232).abs() < 1e-9);
        assert_eq!(ev("max(1, 4, 2)"), 4.0);
        assert!((ev("2 * pi") - std::f64::consts::TAU).abs() < 1e-12);
        assert_eq!(ev("1.5e2"), 150.0);
    }

    #[test]
    fn references() {
        assert_eq!(ev("width / 2 + 3"), 13.0);
        assert_eq!(ev("Pad.Length * 2"), 10.0);
        assert_eq!(ev("Sketch.Constraints[3]"), 7.0);
        assert_eq!(ev("<<My Sketch>>.w"), 2.0);
        assert!(parse("1 +").is_err());
        assert!(parse("foo(").is_err());
    }

    #[test]
    fn dependency_order_and_cycles() {
        let mut m = BTreeMap::new();
        m.insert("a".to_string(), parse("b * 2").unwrap());
        m.insert("b".to_string(), parse("ext + 1").unwrap());
        m.insert("x".to_string(), parse("y").unwrap());
        m.insert("y".to_string(), parse("x").unwrap());
        let (vals, errs) = eval_all(&m, &mut |n| (n == "ext").then_some(4.0), &|n| Some(n.to_string()));
        assert_eq!(vals["a"], 10.0);
        assert_eq!(vals["b"], 5.0);
        assert!(errs.contains_key("x") && errs.contains_key("y"));
    }
}
