//! OpenGL renderer (lazy-initialised inside the egui paint callback, so it always
//! runs with the right context current).
use eframe::glow::{self, HasContext};
use std::sync::Arc;

pub struct ObjUpload {
    /// pos(3)+normal(3)+color(3) interleaved
    pub verts: Vec<f32>,
    pub indices: Vec<u32>,
    /// line-segment vertices (see `push_segment`)
    pub edge_verts: Vec<f32>,
    /// Use the per-vertex colors instead of the object color.
    pub vcolor: bool,
}

#[derive(Default)]
pub struct HlData {
    pub hover_tris: Vec<f32>, // pos+normal, unindexed
    pub sel_tris: Vec<f32>,
    pub lines: Vec<f32>,
}

pub const HOVER_COLOR: [f32; 4] = [0.93, 0.88, 0.10, 1.0];
pub const SEL_COLOR: [f32; 4] = [0.11, 0.70, 0.11, 1.0];
pub const LINE_STRIDE: usize = 13;

/// Appends one thick screen-space segment (6 vertices) to `out`.
pub fn push_segment(out: &mut Vec<f32>, a: [f32; 3], b: [f32; 3], color: [f32; 4], width: f32) {
    for (t, s) in [(0.0, -1.0), (1.0, -1.0), (1.0, 1.0), (0.0, -1.0), (1.0, 1.0), (0.0, 1.0)] {
        out.extend_from_slice(&a);
        out.extend_from_slice(&b);
        out.extend_from_slice(&[t, s]);
        out.extend_from_slice(&color);
        out.push(width);
    }
}

pub struct Frame {
    pub vp: [f32; 16],
    pub view3: [f32; 9],
    pub bg: ([f32; 3], [f32; 3]),
    pub objs: Arc<Vec<ObjUpload>>,
    pub obj_gen: u64,
    pub vis: Vec<(bool, [f32; 4])>,
    pub hl: Arc<HlData>,
    pub hl_gen: u64,
    pub overlay: Arc<Vec<f32>>,
    pub ov_gen: u64,
}

struct LineBuf {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    count: i32,
}
struct TriBuf {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    count: i32,
}
struct GObj {
    vao: glow::VertexArray,
    vbo: glow::Buffer,
    ebo: glow::Buffer,
    n_idx: i32,
    edges: LineBuf,
    vcolor: bool,
}

pub struct Gpu {
    mesh_prog: glow::Program,
    mesh_vp: Option<glow::UniformLocation>,
    mesh_v3: Option<glow::UniformLocation>,
    mesh_col: Option<glow::UniformLocation>,
    mesh_vcol: Option<glow::UniformLocation>,
    line_prog: glow::Program,
    line_vp: Option<glow::UniformLocation>,
    line_vs: Option<glow::UniformLocation>,
    line_bias: Option<glow::UniformLocation>,
    line_ppp: Option<glow::UniformLocation>,
    bg_prog: glow::Program,
    bg_top: Option<glow::UniformLocation>,
    bg_bot: Option<glow::UniformLocation>,
    bg_vao: glow::VertexArray,
    objs: Vec<GObj>,
    obj_gen: u64,
    hl_gen: u64,
    ov_gen: u64,
    hover_tris: TriBuf,
    sel_tris: TriBuf,
    hl_lines: LineBuf,
    ov_lines: LineBuf,
}

const MESH_VS: &str = r#"#version 330 core
layout(location=0) in vec3 a_pos;
layout(location=1) in vec3 a_nrm;
layout(location=2) in vec3 a_col;
uniform mat4 u_vp;
uniform mat3 u_v3;
out vec3 v_n;
out vec3 v_c;
void main(){ v_n = u_v3 * a_nrm; v_c = a_col; gl_Position = u_vp * vec4(a_pos,1.0); }
"#;
const MESH_FS: &str = r#"#version 330 core
in vec3 v_n;
in vec3 v_c;
uniform vec4 u_color;
uniform int u_vcol;
out vec4 frag;
void main(){
  vec4 base = u_vcol == 1 ? vec4(v_c, u_color.a) : u_color;
  vec3 n = normalize(v_n);
  if(n.z < 0.0) n = -n;
  vec3 l1 = normalize(vec3(-0.35,0.45,0.82));
  vec3 l2 = normalize(vec3(0.6,-0.3,0.4));
  float d = max(dot(n,l1),0.0)*0.62 + max(dot(n,l2),0.0)*0.20;
  vec3 h = normalize(l1 + vec3(0.0,0.0,1.0));
  float s = pow(max(dot(n,h),0.0), 40.0) * 0.22;
  if(u_vcol == 1){ d = d*0.55 + 0.25; s *= 0.4; }
  frag = vec4(base.rgb*(0.30+d) + vec3(s), base.a);
}
"#;
const LINE_VS: &str = r#"#version 330 core
layout(location=0) in vec3 a;
layout(location=1) in vec3 b;
layout(location=2) in vec2 corner;
layout(location=3) in vec4 color;
layout(location=4) in float width;
uniform mat4 u_vp;
uniform vec2 u_vs;
uniform float u_bias;
uniform float u_ppp;
out vec4 v_color;
void main(){
  vec4 ca = u_vp*vec4(a,1.0);
  vec4 cb = u_vp*vec4(b,1.0);
  vec4 c = mix(ca,cb,corner.x);
  vec2 sa = ca.xy/ca.w*0.5*u_vs;
  vec2 sb = cb.xy/cb.w*0.5*u_vs;
  vec2 d = sb-sa; float l = length(d);
  d = l>1e-4 ? d/l : vec2(1.0,0.0);
  vec2 n = vec2(-d.y,d.x);
  float wpx = width*u_ppp;
  vec2 off = n*corner.y*wpx*0.5 + d*(corner.x*2.0-1.0)*wpx*0.5;
  gl_Position = c;
  gl_Position.xy += off/(0.5*u_vs)*c.w;
  gl_Position.z -= u_bias*c.w;
  v_color = color;
}
"#;
const LINE_FS: &str = r#"#version 330 core
in vec4 v_color; out vec4 frag;
void main(){ frag = v_color; }
"#;
const BG_VS: &str = r#"#version 330 core
out float v_t;
void main(){
  vec2 p = vec2((gl_VertexID==1)?3.0:-1.0, (gl_VertexID==2)?3.0:-1.0);
  v_t = p.y*0.5+0.5;
  gl_Position = vec4(p,0.999,1.0);
}
"#;
const BG_FS: &str = r#"#version 330 core
in float v_t; uniform vec3 u_top; uniform vec3 u_bot; out vec4 frag;
void main(){ frag = vec4(mix(u_bot,u_top,clamp(v_t,0.0,1.0)),1.0); }
"#;

unsafe fn program(gl: &glow::Context, vs: &str, fs: &str) -> glow::Program {
    unsafe {
        let p = gl.create_program().unwrap();
        let mut shaders = vec![];
        for (ty, src) in [(glow::VERTEX_SHADER, vs), (glow::FRAGMENT_SHADER, fs)] {
            let s = gl.create_shader(ty).unwrap();
            gl.shader_source(s, src);
            gl.compile_shader(s);
            if !gl.get_shader_compile_status(s) {
                panic!("shader error: {}", gl.get_shader_info_log(s));
            }
            gl.attach_shader(p, s);
            shaders.push(s);
        }
        gl.link_program(p);
        if !gl.get_program_link_status(p) {
            panic!("link error: {}", gl.get_program_info_log(p));
        }
        for s in shaders {
            gl.detach_shader(p, s);
            gl.delete_shader(s);
        }
        p
    }
}

unsafe fn line_buf(gl: &glow::Context) -> LineBuf {
    unsafe {
        let vao = gl.create_vertex_array().unwrap();
        let vbo = gl.create_buffer().unwrap();
        gl.bind_vertex_array(Some(vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        let stride = (LINE_STRIDE * 4) as i32;
        for (loc, size, off) in [(0u32, 3, 0), (1, 3, 12), (2, 2, 24), (3, 4, 32), (4, 1, 48)] {
            gl.enable_vertex_attrib_array(loc);
            gl.vertex_attrib_pointer_f32(loc, size, glow::FLOAT, false, stride, off);
        }
        gl.bind_vertex_array(None);
        LineBuf { vao, vbo, count: 0 }
    }
}
unsafe fn tri_buf(gl: &glow::Context) -> TriBuf {
    unsafe {
        let vao = gl.create_vertex_array().unwrap();
        let vbo = gl.create_buffer().unwrap();
        gl.bind_vertex_array(Some(vao));
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        for (loc, off) in [(0u32, 0), (1, 12)] {
            gl.enable_vertex_attrib_array(loc);
            gl.vertex_attrib_pointer_f32(loc, 3, glow::FLOAT, false, 24, off);
        }
        gl.bind_vertex_array(None);
        TriBuf { vao, vbo, count: 0 }
    }
}
unsafe fn fill(gl: &glow::Context, vbo: glow::Buffer, data: &[f32]) {
    unsafe {
        gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
        gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck_f32(data), glow::DYNAMIC_DRAW);
    }
}
fn bytemuck_f32(d: &[f32]) -> &[u8] {
    unsafe { std::slice::from_raw_parts(d.as_ptr() as *const u8, std::mem::size_of_val(d)) }
}
fn bytes_u32(d: &[u32]) -> &[u8] {
    unsafe { std::slice::from_raw_parts(d.as_ptr() as *const u8, std::mem::size_of_val(d)) }
}

impl Gpu {
    pub fn new(gl: &glow::Context) -> Gpu {
        unsafe {
            let mesh_prog = program(gl, MESH_VS, MESH_FS);
            let line_prog = program(gl, LINE_VS, LINE_FS);
            let bg_prog = program(gl, BG_VS, BG_FS);
            Gpu {
                mesh_vp: gl.get_uniform_location(mesh_prog, "u_vp"),
                mesh_v3: gl.get_uniform_location(mesh_prog, "u_v3"),
                mesh_col: gl.get_uniform_location(mesh_prog, "u_color"),
                mesh_vcol: gl.get_uniform_location(mesh_prog, "u_vcol"),
                line_vp: gl.get_uniform_location(line_prog, "u_vp"),
                line_vs: gl.get_uniform_location(line_prog, "u_vs"),
                line_bias: gl.get_uniform_location(line_prog, "u_bias"),
                line_ppp: gl.get_uniform_location(line_prog, "u_ppp"),
                bg_top: gl.get_uniform_location(bg_prog, "u_top"),
                bg_bot: gl.get_uniform_location(bg_prog, "u_bot"),
                mesh_prog,
                line_prog,
                bg_prog,
                bg_vao: gl.create_vertex_array().unwrap(),
                objs: vec![],
                obj_gen: u64::MAX,
                hl_gen: u64::MAX,
                ov_gen: u64::MAX,
                hover_tris: tri_buf(gl),
                sel_tris: tri_buf(gl),
                hl_lines: line_buf(gl),
                ov_lines: line_buf(gl),
            }
        }
    }

    fn upload_objs(&mut self, gl: &glow::Context, objs: &[ObjUpload]) {
        unsafe {
            for o in self.objs.drain(..) {
                gl.delete_vertex_array(o.vao);
                gl.delete_buffer(o.vbo);
                gl.delete_buffer(o.ebo);
                gl.delete_vertex_array(o.edges.vao);
                gl.delete_buffer(o.edges.vbo);
            }
            for u in objs {
                let vao = gl.create_vertex_array().unwrap();
                let vbo = gl.create_buffer().unwrap();
                let ebo = gl.create_buffer().unwrap();
                gl.bind_vertex_array(Some(vao));
                gl.bind_buffer(glow::ARRAY_BUFFER, Some(vbo));
                gl.buffer_data_u8_slice(glow::ARRAY_BUFFER, bytemuck_f32(&u.verts), glow::STATIC_DRAW);
                gl.bind_buffer(glow::ELEMENT_ARRAY_BUFFER, Some(ebo));
                gl.buffer_data_u8_slice(glow::ELEMENT_ARRAY_BUFFER, bytes_u32(&u.indices), glow::STATIC_DRAW);
                for (loc, off) in [(0u32, 0), (1, 12), (2, 24)] {
                    gl.enable_vertex_attrib_array(loc);
                    gl.vertex_attrib_pointer_f32(loc, 3, glow::FLOAT, false, 36, off);
                }
                gl.bind_vertex_array(None);
                let mut edges = line_buf(gl);
                fill(gl, edges.vbo, &u.edge_verts);
                edges.count = (u.edge_verts.len() / LINE_STRIDE) as i32;
                self.objs.push(GObj { vao, vbo, ebo, n_idx: u.indices.len() as i32, edges, vcolor: u.vcolor });
            }
        }
    }

    pub fn paint(&mut self, gl: &glow::Context, f: &Frame, viewport_px: [f32; 2], ppp: f32) {
        if self.obj_gen != f.obj_gen {
            self.upload_objs(gl, &f.objs);
            self.obj_gen = f.obj_gen;
        }
        unsafe {
            if self.hl_gen != f.hl_gen {
                for (tb, d) in [(&mut self.hover_tris, &f.hl.hover_tris), (&mut self.sel_tris, &f.hl.sel_tris)] {
                    fill(gl, tb.vbo, d);
                    tb.count = (d.len() / 6) as i32;
                }
                fill(gl, self.hl_lines.vbo, &f.hl.lines);
                self.hl_lines.count = (f.hl.lines.len() / LINE_STRIDE) as i32;
                self.hl_gen = f.hl_gen;
            }
            if self.ov_gen != f.ov_gen {
                fill(gl, self.ov_lines.vbo, &f.overlay);
                self.ov_lines.count = (f.overlay.len() / LINE_STRIDE) as i32;
                self.ov_gen = f.ov_gen;
            }

            gl.depth_mask(true);
            gl.clear_depth_f32(1.0);
            gl.clear(glow::DEPTH_BUFFER_BIT);

            // background gradient
            gl.disable(glow::DEPTH_TEST);
            gl.disable(glow::BLEND);
            gl.use_program(Some(self.bg_prog));
            gl.uniform_3_f32_slice(self.bg_top.as_ref(), &f.bg.0);
            gl.uniform_3_f32_slice(self.bg_bot.as_ref(), &f.bg.1);
            gl.bind_vertex_array(Some(self.bg_vao));
            gl.draw_arrays(glow::TRIANGLES, 0, 3);

            gl.enable(glow::DEPTH_TEST);
            gl.depth_func(glow::LEQUAL);
            gl.disable(glow::CULL_FACE);

            // solids
            gl.use_program(Some(self.mesh_prog));
            gl.uniform_matrix_4_f32_slice(self.mesh_vp.as_ref(), false, &f.vp);
            gl.uniform_matrix_3_f32_slice(self.mesh_v3.as_ref(), false, &f.view3);
            gl.enable(glow::POLYGON_OFFSET_FILL);
            gl.polygon_offset(1.0, 1.0);
            for transparent in [false, true] {
                if transparent {
                    gl.enable(glow::BLEND);
                    gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
                    gl.depth_mask(false);
                }
                for (o, (vis, col)) in self.objs.iter().zip(&f.vis) {
                    if !*vis || (col[3] < 0.999) != transparent {
                        continue;
                    }
                    gl.uniform_4_f32_slice(self.mesh_col.as_ref(), col);
                    gl.uniform_1_i32(self.mesh_vcol.as_ref(), o.vcolor as i32);
                    gl.bind_vertex_array(Some(o.vao));
                    gl.draw_elements(glow::TRIANGLES, o.n_idx, glow::UNSIGNED_INT, 0);
                }
            }
            gl.depth_mask(true);
            gl.disable(glow::BLEND);
            // face highlights (slightly in front of the shaded faces)
            gl.polygon_offset(0.0, 0.0);
            gl.uniform_1_i32(self.mesh_vcol.as_ref(), 0);
            for (tb, col) in [(&self.sel_tris, SEL_COLOR), (&self.hover_tris, HOVER_COLOR)] {
                if tb.count > 0 {
                    gl.uniform_4_f32_slice(self.mesh_col.as_ref(), &col);
                    gl.bind_vertex_array(Some(tb.vao));
                    gl.draw_arrays(glow::TRIANGLES, 0, tb.count);
                }
            }
            gl.disable(glow::POLYGON_OFFSET_FILL);

            // lines
            gl.use_program(Some(self.line_prog));
            gl.uniform_matrix_4_f32_slice(self.line_vp.as_ref(), false, &f.vp);
            gl.uniform_2_f32(self.line_vs.as_ref(), viewport_px[0], viewport_px[1]);
            gl.uniform_1_f32(self.line_ppp.as_ref(), ppp);
            gl.enable(glow::BLEND);
            gl.blend_func(glow::SRC_ALPHA, glow::ONE_MINUS_SRC_ALPHA);
            gl.uniform_1_f32(self.line_bias.as_ref(), 3e-4);
            for (o, (vis, _)) in self.objs.iter().zip(&f.vis) {
                if *vis && o.edges.count > 0 {
                    gl.bind_vertex_array(Some(o.edges.vao));
                    gl.draw_arrays(glow::TRIANGLES, 0, o.edges.count);
                }
            }
            if self.hl_lines.count > 0 {
                gl.uniform_1_f32(self.line_bias.as_ref(), 6e-4);
                gl.bind_vertex_array(Some(self.hl_lines.vao));
                gl.draw_arrays(glow::TRIANGLES, 0, self.hl_lines.count);
            }
            if self.ov_lines.count > 0 {
                gl.disable(glow::DEPTH_TEST);
                gl.uniform_1_f32(self.line_bias.as_ref(), 0.0);
                gl.bind_vertex_array(Some(self.ov_lines.vao));
                gl.draw_arrays(glow::TRIANGLES, 0, self.ov_lines.count);
            }

            gl.bind_vertex_array(None);
            gl.use_program(None);
            gl.disable(glow::DEPTH_TEST);
            gl.depth_mask(true);
        }
    }
}
