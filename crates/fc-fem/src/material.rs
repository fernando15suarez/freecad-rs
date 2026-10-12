//! Solid material cards in the shape of FreeCAD's material library (`*.FCMat`).
//!
//! Units are those of the FreeCAD cards (SI with MPa for stiffness); the CalculiX writer converts
//! them to the mm / t / s / K system FreeCAD's ccx writer uses.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Material {
    /// Card name, e.g. "Steel-Generic" (written as a comment, like FreeCAD).
    pub name: String,
    /// Young's modulus in MPa.
    pub youngs_modulus: f64,
    pub poisson_ratio: f64,
    /// Density in kg/m^3.
    pub density: f64,
    /// Thermal conductivity in W/m/K.
    #[serde(default)]
    pub thermal_conductivity: f64,
    /// Linear thermal expansion coefficient in 1/K (m/m/K).
    #[serde(default)]
    pub thermal_expansion: f64,
    /// Specific heat in J/kg/K.
    #[serde(default)]
    pub specific_heat: f64,
    /// Reference temperature for thermal expansion in K (FreeCAD defaults to 0 K when absent).
    #[serde(default)]
    pub expansion_reference_temperature: Option<f64>,
    /// Yield strength in MPa (not used by the solver; for safety factors in the app).
    #[serde(default)]
    pub yield_strength: Option<f64>,
    #[serde(default)]
    pub ultimate_tensile_strength: Option<f64>,
}

impl Material {
    /// FreeCAD "Steel-Generic" (Standard/Metal/Steel/Steel-Generic.FCMat).
    pub fn steel_generic() -> Self {
        Material {
            name: "Steel-Generic".into(),
            youngs_modulus: 200_000.0,
            poisson_ratio: 0.3,
            density: 7900.0,
            thermal_conductivity: 50.0,
            thermal_expansion: 0.000012,
            specific_heat: 500.0,
            expansion_reference_temperature: None,
            yield_strength: None,
            ultimate_tensile_strength: None,
        }
    }
    /// FreeCAD "S235JR" (Steel-S235JR.FCMat).
    pub fn steel_s235jr() -> Self {
        Material {
            name: "S235JR".into(),
            youngs_modulus: 210_000.0,
            poisson_ratio: 0.3,
            density: 7800.0,
            thermal_conductivity: 0.0,
            thermal_expansion: 0.000011,
            specific_heat: 0.0,
            expansion_reference_temperature: None,
            yield_strength: Some(235.0),
            ultimate_tensile_strength: Some(360.0),
        }
    }
    /// FreeCAD "Aluminum 6061-T6" (Aluminum-6061-T6.FCMat).
    pub fn aluminium_6061_t6() -> Self {
        Material {
            name: "Aluminum 6061-T6".into(),
            youngs_modulus: 68_900.0,
            poisson_ratio: 0.33,
            density: 2700.0,
            thermal_conductivity: 167.0,
            thermal_expansion: 0.0000236,
            specific_heat: 896.0,
            expansion_reference_temperature: None,
            yield_strength: Some(276.0),
            ultimate_tensile_strength: Some(310.0),
        }
    }
    /// FreeCAD "PLA" (Thermoplast/PLA-Generic.FCMat).
    pub fn pla() -> Self {
        Material {
            name: "PLA".into(),
            youngs_modulus: 3640.0,
            poisson_ratio: 0.36,
            density: 1240.0,
            thermal_conductivity: 0.13,
            thermal_expansion: 0.000041,
            specific_heat: 1800.0,
            expansion_reference_temperature: None,
            yield_strength: Some(35.9),
            ultimate_tensile_strength: Some(26.4),
        }
    }
    /// FreeCAD "ABS" (Thermoplast/ABS-Generic.FCMat).
    pub fn abs() -> Self {
        Material {
            name: "ABS".into(),
            youngs_modulus: 2300.0,
            poisson_ratio: 0.37,
            density: 1060.0,
            thermal_conductivity: 0.158,
            thermal_expansion: 0.000093,
            specific_heat: 2050.0,
            expansion_reference_temperature: None,
            yield_strength: Some(44.1),
            ultimate_tensile_strength: Some(38.8),
        }
    }
    /// FreeCAD "Copper Generic" (Copper-Generic.FCMat).
    pub fn copper_generic() -> Self {
        Material {
            name: "Copper Generic".into(),
            youngs_modulus: 119_000.0,
            poisson_ratio: 0.343,
            density: 8960.0,
            thermal_conductivity: 398.0,
            thermal_expansion: 0.0000165,
            specific_heat: 385.0,
            expansion_reference_temperature: None,
            yield_strength: Some(33.3),
            ultimate_tensile_strength: Some(210.0),
        }
    }
    /// FreeCAD "Ti-6Al-4V (Grade 5)".
    pub fn ti_6al_4v() -> Self {
        Material {
            name: "Ti-6Al-4V (Grade 5)".into(),
            youngs_modulus: 114_000.0,
            poisson_ratio: 0.33,
            density: 4430.0,
            thermal_conductivity: 6.7,
            thermal_expansion: 0.0000086,
            specific_heat: 553.0,
            expansion_reference_temperature: None,
            yield_strength: Some(910.0),
            ultimate_tensile_strength: Some(975.5),
        }
    }
    /// "CalculiX-Steel" as used by FreeCAD's FEM examples (E = 210 GPa, nu = 0.3).
    pub fn calculix_steel() -> Self {
        Material {
            name: "CalculiX-Steel".into(),
            youngs_modulus: 210_000.0,
            poisson_ratio: 0.3,
            density: 7900.0,
            thermal_conductivity: 43.0,
            thermal_expansion: 0.000012,
            specific_heat: 590.0,
            expansion_reference_temperature: None,
            yield_strength: None,
            ultimate_tensile_strength: None,
        }
    }

    /// All built-in cards.
    pub fn library() -> Vec<Material> {
        vec![
            Self::steel_generic(),
            Self::steel_s235jr(),
            Self::aluminium_6061_t6(),
            Self::pla(),
            Self::abs(),
            Self::copper_generic(),
            Self::ti_6al_4v(),
            Self::calculix_steel(),
        ]
    }

    /// Look a built-in card up by name (case-insensitive; accepts FreeCAD file stems such as
    /// "Aluminum-6061-T6" or "PLA-Generic" too).
    pub fn by_name(name: &str) -> Option<Material> {
        let key = |s: &str| s.to_ascii_lowercase().replace(['-', ' ', '_'], "").replace("aluminium", "aluminum");
        let k = key(name);
        let k2 = k.trim_end_matches("generic").to_string();
        Self::library().into_iter().find(|m| {
            let mk = key(&m.name);
            mk == k || mk == k2 || mk.trim_end_matches("generic") == k2 || (k.starts_with("steels235jr") && mk == "s235jr")
        })
    }

    /// Shear modulus in MPa.
    pub fn shear_modulus(&self) -> f64 {
        self.youngs_modulus / (2.0 * (1.0 + self.poisson_ratio))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lookup() {
        assert_eq!(Material::by_name("Steel-Generic").unwrap().youngs_modulus, 200_000.0);
        assert_eq!(Material::by_name("Aluminium-6061-T6").unwrap().density, 2700.0);
        assert_eq!(Material::by_name("PLA-Generic").unwrap().poisson_ratio, 0.36);
        assert_eq!(Material::by_name("pla").unwrap().name, "PLA");
        assert!(Material::by_name("Unobtainium").is_none());
    }
}
