use std::path::{Path, PathBuf};

const OCCT_LIBS: &[&str] = &[
    "TKDESTEP", "TKDESTL", "TKDEIGES", "TKDE", "TKXSBase", "TKOffset", "TKFillet", "TKBool", "TKBO",
    "TKFeat", "TKMesh", "TKShHealing", "TKPrim", "TKTopAlgo", "TKGeomAlgo", "TKBRep", "TKGeomBase",
    "TKG3d", "TKG2d", "TKMath", "TKernel",
];

/// Bump when `occt-patch/` changes so existing builds pick up the patched sources.
const PATCH_STAMP: &str = ".fcrs-occt-patch-1";

/// OCCT sources shipped inside the occt-sys crate in the cargo registry.
fn occt_src() -> PathBuf {
    let home = std::env::var("CARGO_HOME")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from(std::env::var("HOME").unwrap()).join(".cargo"));
    for reg in std::fs::read_dir(home.join("registry/src")).expect("cargo registry") {
        let dir = reg.unwrap().path().join("occt-sys-7.8.1/OCCT");
        if dir.join("CMakeLists.txt").exists() {
            return dir;
        }
    }
    panic!("occt-sys 7.8.1 sources not found in the cargo registry");
}

/// Same configuration as `occt_sys::build_occt`, plus our source patches (see occt-patch/).
fn build_occt(out: &Path) {
    let patch = Path::new(&std::env::var("CARGO_MANIFEST_DIR").unwrap()).join("occt-patch");
    let mut cfg = cmake::Config::new(occt_src());
    cfg.define("BUILD_PATCH", &patch).define("BUILD_LIBRARY_TYPE", "Static");
    for opt in [
        "BUILD_MODULE_ApplicationFramework", "BUILD_MODULE_Draw", "USE_D3D", "USE_DRACO", "USE_EIGEN",
        "USE_FFMPEG", "USE_FREEIMAGE", "USE_FREETYPE", "USE_GLES2", "USE_OPENGL", "USE_OPENVR",
        "USE_RAPIDJSON", "USE_TBB", "USE_TCL", "USE_TK", "USE_VTK", "USE_XLIB",
    ] {
        cfg.define(opt, "FALSE");
    }
    // cmake 4 dropped compatibility with OCCT's old cmake_minimum_required.
    cfg.define("CMAKE_POLICY_VERSION_MINIMUM", "3.5");
    cfg.define("INSTALL_DIR_LIB", "lib").define("INSTALL_DIR_INCLUDE", "include").profile("Release").out_dir(out).build();
}

fn main() {
    let occt = occt_sys::occt_path();
    if !occt.join("lib").join("libTKernel.a").exists() || !occt.join(PATCH_STAMP).exists() {
        build_occt(&occt);
        std::fs::write(occt.join(PATCH_STAMP), "").unwrap();
    }
    println!("cargo:rerun-if-changed=occt-patch");
    println!("cargo:rustc-link-search=native={}", occt.join("lib").display());
    for lib in OCCT_LIBS {
        println!("cargo:rustc-link-lib=static={lib}");
    }
    cxx_build::bridge("src/ffi.rs")
        .file("cpp/shim.cpp")
        .include(occt.join("include"))
        .include("cpp")
        .std("c++17")
        .flag_if_supported("-Wno-deprecated-declarations")
        .flag_if_supported("-Wno-unused-parameter")
        .compile("fc-occt-shim");
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=cpp/shim.cpp");
    println!("cargo:rerun-if-changed=cpp/shim.h");
}
