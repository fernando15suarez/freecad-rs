//! The whole pipeline: geometry -> BREP -> Gmsh mesh -> CalculiX input -> ccx -> results.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use crate::frd::{parse_dat_frequencies, parse_dat_totals, parse_frd};
use crate::inp::{write_inp, InpOptions};
use crate::mesh::{run_gmsh, FemMesh};
use crate::model::{Analysis, ShapeSource, SubRef};
use crate::resolve::{resolve, FaceAreas};
use crate::results::Results;
use crate::FemError;

/// CalculiX job name; FreeCAD names the input after the mesh object ("Mesh.inp").
pub const JOB_NAME: &str = "Mesh";

/// Options for a run.
#[derive(Debug, Clone, Default)]
pub struct RunOptions {
    /// Working directory to use (created if missing). `None` = a fresh temp directory.
    pub work_dir: Option<PathBuf>,
    /// Keep the temp directory (always kept when `work_dir` is given).
    pub keep_files: bool,
    /// Footer data for the .inp.
    pub inp: Option<InpOptions>,
}

/// Everything a run produced.
#[derive(Debug, Clone)]
pub struct Solution {
    pub mesh: FemMesh,
    pub results: Results,
    /// Directory with shape.brep, shape2mesh.geo, mesh.msh, Mesh.inp, Mesh.frd, Mesh.dat
    /// (deleted at the end unless kept).
    pub work_dir: PathBuf,
    pub gmsh_log: String,
    pub ccx_log: String,
}

static COUNTER: AtomicUsize = AtomicUsize::new(0);

fn temp_dir() -> std::io::Result<PathBuf> {
    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.subsec_nanos()).unwrap_or(0);
    let d = std::env::temp_dir().join(format!("fc-fem-{}-{}-{nanos}", std::process::id(), COUNTER.fetch_add(1, Ordering::Relaxed)));
    std::fs::create_dir_all(&d)?;
    Ok(d)
}

/// Write the analysed shape to `dir/shape.brep`; returns its path and, when the B-rep is
/// available in memory, the exact face areas.
pub fn prepare_geometry(shape: &ShapeSource, dir: &Path) -> Result<(PathBuf, Option<FaceAreas>), FemError> {
    let out = dir.join("shape.brep");
    let areas = |s: &fc_occt::Shape| -> FaceAreas { s.faces().iter().enumerate().map(|(i, f)| (i + 1, f.area)).collect() };
    match shape {
        ShapeSource::Brep(p) => {
            std::fs::copy(p, &out).map_err(|e| FemError::Geometry(format!("cannot read {}: {e}", p.display())))?;
            Ok((out, None))
        }
        ShapeSource::Step(p) => {
            let s = fc_occt::Shape::read_step(&p.to_string_lossy()).map_err(|e| FemError::Geometry(format!("STEP import failed: {e:?}")))?;
            s.write_brep(&out.to_string_lossy()).map_err(|e| FemError::Geometry(format!("BREP export failed: {e:?}")))?;
            Ok((out, Some(areas(&s))))
        }
        ShapeSource::Shape(s) => {
            if s.is_null() || s.solid_count() == 0 {
                return Err(FemError::Geometry("the shape has no solid to analyse".into()));
            }
            s.write_brep(&out.to_string_lossy()).map_err(|e| FemError::Geometry(format!("BREP export failed: {e:?}")))?;
            Ok((out, Some(areas(s))))
        }
    }
}

/// All sub-element references used by an analysis (for the .geo physical groups).
fn all_refs(a: &Analysis) -> Vec<SubRef> {
    a.constraints.iter().flat_map(|c| c.refs.iter().cloned()).collect()
}

/// Mesh the analysis geometry in `dir` (Gmsh).
pub fn mesh_analysis(a: &Analysis, dir: &Path) -> Result<(FemMesh, Option<FaceAreas>, String), FemError> {
    std::fs::create_dir_all(dir)?;
    let (brep, areas) = prepare_geometry(&a.shape, dir)?;
    // refinement references go by index; resolve signatures needs a mesh, so use the index.
    let run = run_gmsh(dir, &brep, &a.mesh, &all_refs(a))?;
    Ok((run.mesh, areas, run.log))
}

/// Write `Mesh.inp` for an already meshed analysis; returns the file path.
pub fn write_input(a: &Analysis, mesh: &FemMesh, areas: Option<&FaceAreas>, dir: &Path, opts: &InpOptions) -> Result<PathBuf, FemError> {
    let resolved = resolve(a, mesh, areas)?;
    let text = write_inp(a, mesh, &resolved, opts);
    let p = dir.join(format!("{JOB_NAME}.inp"));
    std::fs::write(&p, text)?;
    Ok(p)
}

/// Run `ccx -i Mesh` in `dir` (as FreeCAD's ccxtools does) and return its output.
pub fn run_ccx(dir: &Path, threads: u32) -> Result<String, FemError> {
    run_ccx_streaming(dir, threads, &mut |_| {}, &AtomicBool::new(false))
}

/// [`run_ccx`] that hands every output line to `on_line` while ccx runs (for a live log) and
/// kills ccx when `cancel` becomes true (then fails with [`FemError::Cancelled`]).
pub fn run_ccx_streaming(dir: &Path, threads: u32, on_line: &mut dyn FnMut(&str), cancel: &AtomicBool) -> Result<String, FemError> {
    use std::io::{BufRead, BufReader, Read};
    use std::process::Stdio;
    let ccx = crate::tools::ccx()?;
    for ext in ["frd", "dat", "sta", "cvg"] {
        let _ = std::fs::remove_file(dir.join(format!("{JOB_NAME}.{ext}")));
    }
    let mut cmd = Command::new(&ccx);
    cmd.arg("-i").arg(JOB_NAME).current_dir(dir).stdout(Stdio::piped()).stderr(Stdio::piped());
    let n = if threads > 0 { threads as usize } else { std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1) };
    cmd.env("OMP_NUM_THREADS", n.to_string());
    cmd.env("CCX_NPROC_EQUATION_SOLVER", n.to_string());
    let mut child = cmd.spawn().map_err(|e| FemError::ToolFailed { tool: "ccx", detail: format!("could not start {}: {e}", ccx.display()) })?;
    let stdout = child.stdout.take().expect("piped stdout");
    let mut stderr = child.stderr.take().expect("piped stderr");
    let (tx, rx) = std::sync::mpsc::channel::<String>();
    let reader = std::thread::spawn(move || {
        for line in BufReader::new(stdout).lines().map_while(Result::ok) {
            if tx.send(line).is_err() {
                break;
            }
        }
    });
    let err_reader = std::thread::spawn(move || {
        let mut s = String::new();
        let _ = stderr.read_to_string(&mut s);
        s
    });
    let mut log = String::new();
    let mut cancelled = false;
    loop {
        match rx.recv_timeout(std::time::Duration::from_millis(50)) {
            Ok(line) => {
                on_line(&line);
                log.push_str(&line);
                log.push('\n');
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => {}
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => break,
        }
        if !cancelled && cancel.load(Ordering::Relaxed) {
            cancelled = true;
            let _ = child.kill();
        }
    }
    let _ = reader.join();
    let status = child.wait()?;
    log.push_str(&err_reader.join().unwrap_or_default());
    if cancelled {
        return Err(FemError::Cancelled);
    }
    let errors: Vec<&str> = log.lines().filter(|l| l.contains("*ERROR")).collect();
    if !errors.is_empty() || !status.success() || !dir.join(format!("{JOB_NAME}.frd")).exists() {
        let detail = if errors.is_empty() { log.lines().rev().take(15).collect::<Vec<_>>().into_iter().rev().collect::<Vec<_>>().join("\n") } else { errors.join("\n") };
        return Err(FemError::ToolFailed { tool: "ccx", detail });
    }
    Ok(log)
}

/// Read Mesh.frd / Mesh.dat from `dir`.
pub fn read_results(mesh: &FemMesh, dir: &Path) -> Result<Results, FemError> {
    let frd = parse_frd(&std::fs::read_to_string(dir.join(format!("{JOB_NAME}.frd")))?)?;
    let dat = std::fs::read_to_string(dir.join(format!("{JOB_NAME}.dat"))).unwrap_or_default();
    let freqs = parse_dat_frequencies(&dat);
    let reactions = if freqs.is_empty() { parse_dat_totals(&dat) } else { Default::default() };
    Results::from_frd(mesh, &frd, reactions, &freqs)
}

/// Mesh, write, solve and read back an analysis.
pub fn solve(a: &Analysis, opts: &RunOptions) -> Result<Solution, FemError> {
    // fail fast with a clear message before doing any work
    crate::tools::gmsh()?;
    crate::tools::ccx()?;
    let (dir, temp) = match &opts.work_dir {
        Some(d) => {
            std::fs::create_dir_all(d)?;
            (d.clone(), false)
        }
        None => (temp_dir()?, true),
    };
    let result = (|| {
        let (mesh, areas, gmsh_log) = mesh_analysis(a, &dir)?;
        write_input(a, &mesh, areas.as_ref(), &dir, &opts.inp.clone().unwrap_or_default())?;
        let ccx_log = run_ccx(&dir, a.solver.threads)?;
        let results = read_results(&mesh, &dir)?;
        Ok(Solution { mesh, results, work_dir: dir.clone(), gmsh_log, ccx_log })
    })();
    if temp && !opts.keep_files {
        let _ = std::fs::remove_dir_all(&dir);
    }
    result
}
