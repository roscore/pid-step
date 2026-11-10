//! 3D joint view: a geared joint (motor + gearbox + arm) replaying the simulated step.
//! Pure canvas 2D with a small perspective projection — no WebGL, no extra dependencies.
//! Drag to orbit; play / pause, time slider and slow-motion speed.

use std::cell::RefCell;
use std::rc::Rc;

use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d, HtmlCanvasElement, HtmlInputElement, HtmlSelectElement, PointerEvent};

use crate::{doc, palette, tr, ST};
use ps_core::Trace;

type V3 = [f64; 3];

pub struct View {
    pub yaw: f64,
    pub pitch: f64,
    /// Playback position, s (simulation time).
    pub t: f64,
    pub playing: bool,
    last_ts: Option<f64>,
    drag: Option<(f64, f64)>,
}

thread_local! {
    pub static VIEW: RefCell<View> = RefCell::new(View {
        yaw: (-38f64).to_radians(), pitch: 16f64.to_radians(), t: 0.0, playing: true, last_ts: None, drag: None,
    });
}

/// Sample index for time `t` (trace is uniform per control tick).
pub fn index_at(tr: &Trace, t: f64) -> usize {
    tr.t.partition_point(|&x| x <= t).saturating_sub(1).min(tr.t.len().saturating_sub(1))
}

fn speed() -> f64 {
    doc().get_element_by_id("v_speed")
        .and_then(|e| e.dyn_into::<HtmlSelectElement>().ok())
        .and_then(|s| s.value().parse().ok())
        .unwrap_or(0.1)
}

fn slider() -> Option<HtmlInputElement> {
    doc().get_element_by_id("v_time").and_then(|e| e.dyn_into().ok())
}

/// Camera: orbit about the joint axis origin, then a mild perspective.
fn project(p: V3, yaw: f64, pitch: f64) -> (f64, f64, f64) {
    let (sy, cy) = yaw.sin_cos();
    let x = cy * p[0] - sy * p[1];
    let y = sy * p[0] + cy * p[1];
    let (sp, cp) = pitch.sin_cos();
    let depth = y * cp + p[2] * sp; // + = away from camera
    let up = p[2] * cp - y * sp;
    let f = 4.0 / (4.0 + depth); // world is scaled to ~1, camera at distance 4
    (x * f, -up * f, depth)
}

fn arm_tip(theta_deg: f64, len: f64) -> V3 {
    let t = theta_deg.to_radians();
    [len * t.cos(), 0.0, len * t.sin()]
}

/// Points of a circle of radius r around the Y axis at y = y0 (joint axis = world Y).
fn ring(y0: f64, r: f64, n: usize) -> Vec<V3> {
    (0..=n).map(|k| {
        let a = k as f64 / n as f64 * std::f64::consts::TAU;
        [r * a.cos(), y0, r * a.sin()]
    }).collect()
}

pub fn draw_view() {
    let Some(cv) = doc().get_element_by_id("c_view").and_then(|e| e.dyn_into::<HtmlCanvasElement>().ok()) else { return };
    let dpr = web_sys::window().unwrap().device_pixel_ratio();
    let (w, h) = (cv.client_width() as f64, cv.client_height() as f64);
    if cv.width() != (w * dpr) as u32 || cv.height() != (h * dpr) as u32 {
        cv.set_width((w * dpr) as u32);
        cv.set_height((h * dpr) as u32);
    }
    let c: CanvasRenderingContext2d = cv.get_context("2d").unwrap().unwrap().dyn_into().unwrap();
    c.set_transform(dpr, 0.0, 0.0, dpr, 0.0, 0.0).ok();
    c.clear_rect(0.0, 0.0, w, h);
    let pal = palette();
    let (yaw, pitch, t) = VIEW.with(|v| { let v = v.borrow(); (v.yaw, v.pitch, v.t) });
    let sc = h.min(w * 0.8) * 0.48;
    let (cx, cy) = (w * 0.5, h * 0.62);
    let to = |p: V3| { let (x, y, _) = project(p, yaw, pitch); (cx + x * sc, cy + y * sc) };
    let line = |pts: &[V3], col: &str, width: f64| {
        c.set_stroke_style_str(col);
        c.set_line_width(width);
        c.begin_path();
        for (k, p) in pts.iter().enumerate() {
            let (x, y) = to(*p);
            if k == 0 { c.move_to(x, y) } else { c.line_to(x, y) }
        }
        c.stroke();
    };
    ST.with(|st| {
        let st = st.borrow();
        let Some(cur) = st.last.as_ref() else { return };
        let i = index_at(cur, t);
        let th = cur.theta_deg[i];
        let sp = cur.setpoint_deg;
        let len = 1.0;
        c.set_line_cap("round");
        c.set_line_join("round");

        // Floor grid (z = -0.45) for depth cues.
        for k in -4..=4 {
            let f = k as f64 * 0.3;
            line(&[[f, -1.2, -0.45], [f, 1.2, -0.45]], pal.grid, 1.0);
            line(&[[-1.2, f, -0.45], [1.2, f, -0.45]], pal.grid, 1.0);
        }
        // Protractor in the arm plane: 0° reference, target, ±2 % band and the path so far.
        line(&[[0.0; 3], arm_tip(0.0, len * 1.08)], pal.grid, 1.0);
        let arc = |a0: f64, a1: f64, r: f64| -> Vec<V3> {
            let n = 48;
            (0..=n).map(|k| arm_tip(a0 + (a1 - a0) * k as f64 / n as f64, r)).collect()
        };
        let band = 0.02 * sp.abs();
        line(&arc(sp - band, sp + band, len * 1.12), pal.band, 9.0);
        c.set_line_dash(&crate::js_sys_array(&[5.0, 4.0])).ok();
        line(&[[0.0; 3], arm_tip(sp, len * 1.12)], pal.sp, 1.5);
        c.set_line_dash(&crate::js_sys_array(&[])).ok();
        let (lo, hi) = cur.theta_deg[..=i].iter().fold((0f64, 0f64), |(a, b), &x| (a.min(x), b.max(x)));
        line(&arc(lo, hi, len * 0.35), pal.main, 2.0);
        // Angle labels: 0° and target.
        c.set_font("11px system-ui, sans-serif");
        c.set_fill_style_str(pal.ghost);
        let (x0, y0) = to(arm_tip(0.0, len * 1.18));
        c.fill_text("0°", x0, y0).ok();
        c.set_fill_style_str(pal.sp);
        let (x1, y1) = to(arm_tip(sp, len * 1.2));
        c.fill_text(&format!("{:.0}°", sp), x1, y1).ok();

        // Pinned run: grey ghost arm at the same time.
        if let Some(g) = st.ghost.as_ref() {
            let gi = index_at(g, t);
            line(&[[0.0; 3], arm_tip(g.theta_deg[gi], len)], pal.ghost, 5.0);
        }

        // Motor (cylinder behind the joint along -Y) and gearbox (short, wider).
        let sat = cur.saturated[i];
        let motor_col = if sat { "#e03131" } else { pal.fg };
        for (y0, y1, r, col) in [(-0.85, -0.4, 0.13, motor_col), (-0.4, -0.12, 0.19, pal.fg)] {
            let a = ring(y0, r, 32);
            let b = ring(y1, r, 32);
            line(&a, col, 1.2);
            line(&b, col, 1.2);
            for k in (0..32).step_by(4) {
                line(&[a[k], b[k]], col, 0.8);
            }
        }
        // Rotor marker on the motor's back face spins N× faster (mod 360°): watch it alias at speed.
        let n_ratio = crate::num("n");
        let rot = (th * if n_ratio.is_finite() { n_ratio } else { 1.0 }).to_radians();
        line(&[[0.0, -0.85, 0.0], [0.11 * rot.cos(), -0.85, 0.11 * rot.sin()]], pal.sp, 2.5);
        // Output shaft and arm.
        line(&[[0.0, -0.12, 0.0], [0.0, 0.08, 0.0]], pal.fg, 4.0);
        line(&[[0.0, 0.05, 0.0], arm_tip(th, len)], pal.main, 8.0);
        // Payload block at the tip.
        let tip = arm_tip(th, len);
        let (tx, ty) = to([tip[0], 0.05, tip[2]]);
        c.set_fill_style_str(pal.main);
        c.begin_path();
        c.arc(tx, ty, 7.0, 0.0, std::f64::consts::TAU).ok();
        c.fill();

        // HUD.
        c.set_font("12px system-ui, sans-serif");
        c.set_fill_style_str(pal.fg);
        let hud = [
            format!("t = {:.1} ms", cur.t[i] * 1e3),
            format!("θ = {:.2}° / {:.1}°", th, sp),
            format!("i = {:.2} A{}", cur.current[i], if sat { format!("  {}", tr("v_sat")) } else { String::new() }),
        ];
        for (k, s) in hud.iter().enumerate() {
            c.fill_text(s, 8.0, 16.0 + 15.0 * k as f64).ok();
        }
        c.set_fill_style_str(pal.ghost);
        c.fill_text(tr("v_hint"), 8.0, h - 8.0).ok();
    });
}

/// Restart playback for a new result.
pub fn reset() {
    VIEW.with(|v| { let mut v = v.borrow_mut(); v.t = 0.0; v.playing = true; v.last_ts = None; });
    sync_controls();
}

fn duration() -> f64 {
    ST.with(|s| s.borrow().last.as_ref().and_then(|t| t.t.last().copied())).unwrap_or(0.0)
}

fn sync_controls() {
    let (t, playing) = VIEW.with(|v| { let v = v.borrow(); (v.t, v.playing) });
    let d = duration();
    if let Some(s) = slider() {
        s.set_value(&format!("{}", if d > 0.0 { (t / d * 1000.0).round() } else { 0.0 }));
    }
    crate::set_text("v_play", tr(if playing { "v_pause" } else { "v_play" }));
}

fn frame(ts: f64) {
    let d = duration();
    let mut redraw_plots = false;
    VIEW.with(|v| {
        let mut v = v.borrow_mut();
        if v.playing && d > 0.0 {
            if let Some(prev) = v.last_ts {
                let dt = ((ts - prev) / 1000.0).min(0.1);
                v.t += dt * speed();
                if v.t > d {
                    v.t = d;
                    v.playing = false;
                }
                redraw_plots = true;
            }
            v.last_ts = Some(ts);
        } else {
            v.last_ts = None;
        }
    });
    if redraw_plots {
        sync_controls();
        crate::draw();
    }
}

pub fn install() {
    // requestAnimationFrame loop.
    let f: Rc<RefCell<Option<Closure<dyn FnMut(f64)>>>> = Rc::new(RefCell::new(None));
    let g = f.clone();
    *g.borrow_mut() = Some(Closure::new(move |ts: f64| {
        frame(ts);
        let w = web_sys::window().unwrap();
        w.request_animation_frame(f.borrow().as_ref().unwrap().as_ref().unchecked_ref()).ok();
    }));
    web_sys::window().unwrap()
        .request_animation_frame(g.borrow().as_ref().unwrap().as_ref().unchecked_ref()).ok();

    crate::on("v_play", "click", || {
        let d = duration();
        VIEW.with(|v| {
            let mut v = v.borrow_mut();
            if !v.playing && v.t >= d { v.t = 0.0; }
            v.playing = !v.playing;
            v.last_ts = None;
        });
        sync_controls();
        crate::draw();
    });
    crate::on("v_time", "input", || {
        let d = duration();
        let x: f64 = slider().map(|s| s.value()).and_then(|s| s.parse().ok()).unwrap_or(0.0);
        VIEW.with(|v| { let mut v = v.borrow_mut(); v.t = d * x / 1000.0; v.playing = false; });
        sync_controls();
        crate::draw();
    });

    // Drag to orbit (pointer events cover mouse and touch).
    let cv: HtmlCanvasElement = doc().get_element_by_id("c_view").unwrap().dyn_into().unwrap();
    let down = Closure::<dyn FnMut(PointerEvent)>::new(|e: PointerEvent| {
        VIEW.with(|v| v.borrow_mut().drag = Some((e.client_x() as f64, e.client_y() as f64)));
        if let Some(t) = e.target().and_then(|t| t.dyn_into::<web_sys::Element>().ok()) {
            t.set_pointer_capture(e.pointer_id()).ok();
        }
    });
    let mv = Closure::<dyn FnMut(PointerEvent)>::new(|e: PointerEvent| {
        let moved = VIEW.with(|v| {
            let mut v = v.borrow_mut();
            let Some((x0, y0)) = v.drag else { return false };
            let (x, y) = (e.client_x() as f64, e.client_y() as f64);
            v.yaw += (x - x0) * 0.01;
            v.pitch = (v.pitch + (y - y0) * 0.01).clamp(-1.2, 1.4);
            v.drag = Some((x, y));
            true
        });
        if moved { draw_view(); }
    });
    let up = Closure::<dyn FnMut(PointerEvent)>::new(|_e: PointerEvent| {
        VIEW.with(|v| v.borrow_mut().drag = None);
    });
    for (ev, cb) in [("pointerdown", down.as_ref()), ("pointermove", mv.as_ref()), ("pointerup", up.as_ref()), ("pointercancel", up.as_ref())] {
        cv.add_event_listener_with_callback(ev, cb.unchecked_ref()).unwrap();
    }
    down.forget();
    mv.forget();
    up.forget();
}
