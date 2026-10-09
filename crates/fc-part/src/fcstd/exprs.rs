//! Rewriting FreeCAD ExpressionEngine expressions into fc-part expression syntax.
//!
//! FreeCAD references look like `Pad.Length`, `Sketch.Constraints.width`,
//! `.Constraints.width` (own object), `<<Label>>.Prop`, `Spreadsheet.alias` and
//! `Sketch.Constraints[3]`. fc-part understands `Feature.Prop`, `Sketch.Constraints.name`,
//! `Sketch.Constraints[i]` (our indices) and bare parameter names.

/// Resolves FreeCAD paths for the rewriter.
pub(crate) trait Resolver {
    /// Object name for a `<<label>>` reference.
    fn by_label(&self, label: &str) -> Option<String>;
    /// Rewrite a dotted path (`segs[0]` empty = the owner object). `None` = keep as written.
    fn path(&self, segs: &[String], owner: &str) -> Option<String>;
}

fn is_ident_start(c: char) -> bool {
    c.is_alphabetic() || c == '_'
}
fn is_ident(c: char) -> bool {
    c.is_alphanumeric() || c == '_'
}

/// Rewrite all references of `src`. The leading `=` of spreadsheet cells is dropped.
pub(crate) fn rewrite(src: &str, owner: &str, r: &dyn Resolver) -> String {
    let s: Vec<char> = src.trim().trim_start_matches('=').chars().collect();
    let mut out = String::new();
    let mut i = 0;
    // Whether the previous significant token was a number (then an identifier is a unit).
    let mut after_number = false;
    while i < s.len() {
        let c = s[i];
        if c.is_whitespace() {
            out.push(c);
            i += 1;
            continue;
        }
        if c.is_ascii_digit() || (c == '.' && s.get(i + 1).is_some_and(|d| d.is_ascii_digit()) && !after_ident(&out)) {
            while i < s.len() && (s[i].is_ascii_digit() || s[i] == '.' || s[i] == 'e' || s[i] == 'E'
                || ((s[i] == '+' || s[i] == '-') && i > 0 && (s[i - 1] == 'e' || s[i - 1] == 'E')))
            {
                // 'e' only as exponent when followed by a digit or sign.
                if (s[i] == 'e' || s[i] == 'E') && !s.get(i + 1).is_some_and(|d| d.is_ascii_digit() || *d == '+' || *d == '-') {
                    break;
                }
                out.push(s[i]);
                i += 1;
            }
            after_number = true;
            continue;
        }
        let starts_path = is_ident_start(c) || (c == '<' && s.get(i + 1) == Some(&'<')) || (c == '.' && s.get(i + 1).is_some_and(|d| is_ident_start(*d)));
        if starts_path {
            // A unit right after a number: copy it.
            if after_number && is_ident_start(c) {
                while i < s.len() && is_ident(s[i]) {
                    out.push(s[i]);
                    i += 1;
                }
                after_number = false;
                continue;
            }
            let mut segs: Vec<String> = Vec::new();
            let mut cur = String::new();
            let mut raw = String::new();
            if c == '.' {
                segs.push(String::new());
                raw.push('.');
                i += 1;
            }
            loop {
                if i + 1 < s.len() && s[i] == '<' && s[i + 1] == '<' {
                    let start = i;
                    i += 2;
                    let mut label = String::new();
                    while i + 1 < s.len() && !(s[i] == '>' && s[i + 1] == '>') {
                        label.push(s[i]);
                        i += 1;
                    }
                    i = (i + 2).min(s.len());
                    raw.extend(&s[start..i]);
                    cur = r.by_label(&label).unwrap_or(label);
                } else {
                    while i < s.len() && is_ident(s[i]) {
                        cur.push(s[i]);
                        raw.push(s[i]);
                        i += 1;
                    }
                }
                // Index suffix: Constraints[3]
                while i < s.len() && s[i] == '[' {
                    while i < s.len() && s[i] != ']' {
                        cur.push(s[i]);
                        raw.push(s[i]);
                        i += 1;
                    }
                    if i < s.len() {
                        cur.push(']');
                        raw.push(']');
                        i += 1;
                    }
                }
                segs.push(std::mem::take(&mut cur));
                if i + 1 < s.len() && s[i] == '.' && (is_ident_start(s[i + 1]) || s[i + 1] == '<') {
                    raw.push('.');
                    i += 1;
                    continue;
                }
                break;
            }
            let merged = segs;
            let is_call = s[i..].iter().find(|c| !c.is_whitespace()) == Some(&'(');
            if is_call && merged.len() == 1 {
                out.push_str(&merged[0]);
            } else {
                match r.path(&merged, owner) {
                    Some(p) => out.push_str(&p),
                    None => out.push_str(&raw),
                }
            }
            after_number = false;
            continue;
        }
        out.push(c);
        after_number = false;
        i += 1;
    }
    out.trim().to_string()
}

fn after_ident(out: &str) -> bool {
    out.chars().last().is_some_and(|c| is_ident(c) || c == ']')
}

#[cfg(test)]
mod tests {
    use super::*;

    struct R;
    impl Resolver for R {
        fn by_label(&self, label: &str) -> Option<String> {
            (label == "My Sheet").then(|| "Spreadsheet".to_string())
        }
        fn path(&self, segs: &[String], owner: &str) -> Option<String> {
            let mut v = segs.to_vec();
            if v[0].is_empty() {
                v[0] = owner.to_string();
            }
            if v[0] == "Spreadsheet" {
                return Some(v[1].clone());
            }
            Some(v.join("."))
        }
    }

    #[test]
    fn rewrites_paths() {
        assert_eq!(rewrite("Spreadsheet.thick * 1 mm + Sketch.Constraints.width / 8", "Pad", &R), "thick * 1 mm + Sketch.Constraints.width / 8");
        assert_eq!(rewrite(".Constraints.width / 2", "Sketch", &R), "Sketch.Constraints.width / 2");
        assert_eq!(rewrite("<<My Sheet>>.depth*1mm/3", "Pocket", &R), "depth*1mm/3");
        assert_eq!(rewrite("=thick * 2 + 1", "Spreadsheet", &R), "thick * 2 + 1");
        assert_eq!(rewrite("sqrt(Pad.Length) + 2.5e-1", "X", &R), "sqrt(Pad.Length) + 2.5e-1");
        assert_eq!(rewrite("Sketch.Constraints[3] * 2", "X", &R), "Sketch.Constraints[3] * 2");
        assert_eq!(rewrite("30 °", "X", &R), "30 °");
    }
}
