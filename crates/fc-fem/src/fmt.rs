//! Number formatting that reproduces what FreeCAD's Python/C++ writers emit.

/// C `%.{p}g` (lowercase) / Python `{:.{p}G}` (uppercase) general format.
pub fn fmt_g(v: f64, p: usize, upper: bool) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0".into() } else { "0".into() };
    }
    if !v.is_finite() {
        return if v.is_nan() { "nan".into() } else if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    let p = p.max(1);
    let s = format!("{:.*e}", p - 1, v);
    let (mant, exp) = s.split_once('e').unwrap();
    let x: i32 = exp.parse().unwrap();
    if x >= -4 && x < p as i32 {
        let decimals = (p as i32 - 1 - x).max(0) as usize;
        strip_zeros(format!("{:.*}", decimals, v))
    } else {
        let m = strip_zeros(mant.to_string());
        format!("{m}{}{}{:02}", if upper { 'E' } else { 'e' }, if x < 0 { '-' } else { '+' }, x.abs())
    }
}

fn strip_zeros(s: String) -> String {
    if s.contains('.') {
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    } else {
        s
    }
}

/// Python `{:.13G}` (FreeCAD's ccx writer "floats read from ccx").
pub fn g13(v: f64) -> String {
    fmt_g(v, 13, true)
}

/// Python `repr(float)` / `str(float)`.
pub fn py(v: f64) -> String {
    if v == 0.0 {
        return if v.is_sign_negative() { "-0.0".into() } else { "0.0".into() };
    }
    let a = v.abs();
    if (1e-4..1e16).contains(&a) {
        let s = format!("{v}");
        if s.contains('.') {
            s
        } else {
            s + ".0"
        }
    } else {
        let s = format!("{v:e}");
        let (m, e) = s.split_once('e').unwrap();
        let ei: i32 = e.parse().unwrap();
        format!("{m}e{}{:02}", if ei < 0 { '-' } else { '+' }, ei.abs())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn python_g13() {
        assert_eq!(g13(200000.0), "200000");
        assert_eq!(g13(0.3), "0.3");
        assert_eq!(g13(1.2e-5), "1.2E-05");
        assert_eq!(g13(7.9e-9), "7.9E-09");
        assert_eq!(g13(9806.65), "9806.65");
        assert_eq!(g13(-1.0), "-1");
        assert_eq!(g13(-0.0), "-0");
        assert_eq!(g13(500000000.0), "500000000");
        assert_eq!(g13(-20.337301587301587), "-20.3373015873");
        assert_eq!(g13(6.444978076265123), "6.444978076265");
        assert_eq!(g13(1e22), "1E+22");
        assert_eq!(g13(0.0001), "0.0001");
        assert_eq!(g13(0.00001), "1E-05");
    }
    #[test]
    fn c_g13() {
        assert_eq!(fmt_g(109.88830826234, 13, false), "109.8883082623");
        assert_eq!(fmt_g(12.5, 13, false), "12.5");
        assert_eq!(fmt_g(1e-15, 13, false), "1e-15");
    }
    #[test]
    fn python_repr() {
        assert_eq!(py(0.1), "0.1");
        assert_eq!(py(293.0), "293.0");
        assert_eq!(py(1e-5), "1e-05");
        assert_eq!(py(1e6), "1000000.0");
        assert_eq!(py(3333.3333333333335), "3333.3333333333335");
        assert_eq!(py(-2.0), "-2.0");
        assert_eq!(py(1e22), "1e+22");
    }
}
