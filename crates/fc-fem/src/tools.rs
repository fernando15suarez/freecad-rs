//! Locating the external programs FreeCAD's FEM workbench drives: Gmsh (mesher) and
//! CalculiX `ccx` (solver).
//!
//! Lookup order for each tool:
//! 1. the environment variable (`FCRS_GMSH` / `FCRS_CCX`), a full path to the binary;
//! 2. the program name on `PATH` (`gmsh`, `ccx`);
//! 3. the binaries bundled with FreeCAD.app on macOS
//!    (`/Applications/FreeCAD.app/Contents/Resources/bin/{gmsh,ccx}`), and the usual
//!    Linux/Windows install locations of FreeCAD.

use std::path::{Path, PathBuf};

use crate::FemError;

/// Which external tool.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Gmsh,
    Ccx,
}

impl Tool {
    pub fn env_var(self) -> &'static str {
        match self {
            Tool::Gmsh => "FCRS_GMSH",
            Tool::Ccx => "FCRS_CCX",
        }
    }
    pub fn program(self) -> &'static str {
        match self {
            Tool::Gmsh => "gmsh",
            Tool::Ccx => "ccx",
        }
    }
    fn bundle_candidates(self) -> Vec<PathBuf> {
        let p = self.program();
        let mut v = vec![
            PathBuf::from(format!("/Applications/FreeCAD.app/Contents/Resources/bin/{p}")),
            PathBuf::from(format!("/usr/lib/freecad/bin/{p}")),
            PathBuf::from(format!("/usr/local/lib/freecad/bin/{p}")),
        ];
        if let Some(home) = std::env::var_os("HOME") {
            v.push(Path::new(&home).join(format!("Applications/FreeCAD.app/Contents/Resources/bin/{p}")));
        }
        if cfg!(windows) {
            for base in ["C:\\Program Files\\FreeCAD 1.1\\bin", "C:\\Program Files\\FreeCAD 1.0\\bin"] {
                v.push(Path::new(base).join(format!("{p}.exe")));
            }
        }
        v
    }
}

fn is_executable(p: &Path) -> bool {
    p.is_file()
}

fn which(program: &str) -> Option<PathBuf> {
    let path = std::env::var_os("PATH")?;
    for dir in std::env::split_paths(&path) {
        let cand = dir.join(program);
        if is_executable(&cand) {
            return Some(cand);
        }
        if cfg!(windows) {
            let cand = dir.join(format!("{program}.exe"));
            if is_executable(&cand) {
                return Some(cand);
            }
        }
    }
    None
}

/// Find a tool, or explain where we looked.
pub fn find(tool: Tool) -> Result<PathBuf, FemError> {
    let mut tried = Vec::new();
    if let Some(v) = std::env::var_os(tool.env_var()) {
        let p = PathBuf::from(&v);
        if is_executable(&p) {
            return Ok(p);
        }
        // An explicit setting that points nowhere is an error, not a reason to fall back silently.
        return Err(FemError::ToolMissing {
            tool: tool.program(),
            detail: format!("{}={} does not exist or is not a file", tool.env_var(), p.display()),
        });
    }
    tried.push(format!("${} (unset)", tool.env_var()));
    if let Some(p) = which(tool.program()) {
        return Ok(p);
    }
    tried.push(format!("`{}` on PATH", tool.program()));
    for c in tool.bundle_candidates() {
        if is_executable(&c) {
            return Ok(c);
        }
        tried.push(c.display().to_string());
    }
    Err(FemError::ToolMissing {
        tool: tool.program(),
        detail: format!(
            "looked in: {}. Install FreeCAD (it bundles gmsh and ccx) or set {} to the binary",
            tried.join(", "),
            tool.env_var()
        ),
    })
}

/// Path to Gmsh.
pub fn gmsh() -> Result<PathBuf, FemError> {
    find(Tool::Gmsh)
}

/// Path to CalculiX ccx.
pub fn ccx() -> Result<PathBuf, FemError> {
    find(Tool::Ccx)
}

/// True when both external tools are available (tests use this to skip solver runs).
pub fn available() -> bool {
    gmsh().is_ok() && ccx().is_ok()
}
