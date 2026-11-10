//! Browser UI for pid-step: reads the form, runs ps_core, draws two canvas plots.
//! Fully client-side; the only stored setting is the language (localStorage, optional).

use std::cell::RefCell;

use ps_core::{metrics, simulate, suggest_gains, verdict, Pid, Plant, Reason, SimCfg, Trace, Verdict};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::{CanvasRenderingContext2d, Document, HtmlCanvasElement, HtmlElement, HtmlInputElement};

#[derive(Default)]
struct State {
    ko: bool,
    last: Option<Trace>,
    ghost: Option<Trace>,
}

thread_local! {
    static ST: RefCell<State> = RefCell::new(State::default());
}

/// (key, Korean, English) for texts set from Rust or via data-i18n.
const TEXT: &[(&str, &str, &str)] = &[
    ("tagline", "감속기 달린 로봇 관절의 PID 위치 스텝 응답과 포화 여부를 확인합니다.", "Geared robot joint: PID position-step response and saturation check."),
    ("plant", "플랜트 (감속 관절 1개)", "Plant (one geared joint)"),
    ("preset", "프리셋", "Preset"),
    ("servo", "소형 서보 관절 (일반적 값)", "Small servo joint (typical)"),
    ("knee", "휴머노이드 무릎 관절 (일반적 값)", "Humanoid knee joint (typical)"),
    ("j_rotor", "로터 관성 J_rotor (kg·m²)", "Rotor inertia J_rotor (kg·m²)"),
    ("kt", "토크 상수 Kt (N·m/A)", "Torque constant Kt (N·m/A)"),
    ("r", "권선 저항 R (Ω)", "Winding resistance R (Ω)"),
    ("v", "전원 전압 (V)", "Supply voltage (V)"),
    ("imax", "최대 전류 (A)", "Max current (A)"),
    ("n", "감속비 N", "Gear ratio N"),
    ("eta", "감속기 효율 η (0–1)", "Gear efficiency η (0–1)"),
    ("j_load", "부하 관성 J_load (kg·m²)", "Load inertia J_load (kg·m²)"),
    ("b", "점성 마찰 b (N·m·s/rad)", "Viscous friction b (N·m·s/rad)"),
    ("tau_c", "쿨롱 마찰 (N·m)", "Coulomb friction (N·m)"),
    ("tau_g", "중력 토크 (N·m)", "Gravity torque (N·m)"),
    ("max_rpm", "모터 최대 속도 (rpm)", "Max motor speed (rpm)"),
    ("ctrl", "PID 제어기 (출력 = 전류, A)", "PID controller (output = current, A)"),
    ("fc", "D 필터 차단 주파수 (Hz, 0 = 끔)", "D filter cutoff (Hz, 0 = off)"),
    ("dmeas", "측정값 미분 (D on measurement)", "Derivative on measurement"),
    ("fs", "샘플링 주파수 (Hz)", "Sample rate (Hz)"),
    ("enc", "출력축 엔코더 카운트/회전 (0 = 이상적)", "Encoder counts/rev at output (0 = ideal)"),
    ("sim", "스텝 테스트", "Step test"),
    ("step", "스텝 크기 (deg)", "Step size (deg)"),
    ("dur", "시뮬레이션 시간 (s)", "Duration (s)"),
    ("suggest", "게인 제안 (극배치)", "Suggest gains (pole placement)"),
    ("bw", "대역폭 ω_n (Hz)", "Bandwidth ω_n (Hz)"),
    ("zeta", "감쇠비 ζ", "Damping ζ"),
    ("apply", "적용", "Apply"),
    ("pin", "현재 결과 고정", "Pin current run"),
    ("unpin", "고정 해제", "Clear pinned"),
    ("model", "모델: 출력축 기준 강체 관절, J_eff = J_load + N²·J_rotor, τ = N·η·Kt·i. 전류는 최대 전류와 역기전력 여유 (V − Kt·ω_motor)/R로 제한. 인덕턴스 무시, 백래시·컴플라이언스 없음.", "Model: rigid joint at the output shaft, J_eff = J_load + N²·J_rotor, τ = N·η·Kt·i. Current limited by max current and back-EMF headroom (V − Kt·ω_motor)/R. Electrical inductance neglected; no backlash or compliance."),
    ("disclaimer", "추정치일 뿐입니다 — 실제 하드웨어에서 안전한 제한값으로 검증하세요.", "Estimates only — validate on hardware with safe limits."),
    ("ok", "✅ 양호", "✅ OK"),
    ("warn", "⚠️ 주의", "⚠️ Check"),
    ("bad", "❌ 위험", "❌ Problem"),
    ("r_unstable", "불안정/발산", "unstable / diverging"),
    ("r_overspeed", "모터 속도 한계 초과", "motor speed over limit"),
    ("r_overshoot", "오버슈트 > 10 %", "overshoot > 10 %"),
    ("r_sat", "포화 시간 > 20 %", "saturated > 20 % of the time"),
    ("r_settle", "정착하지 않음", "not settled"),
    ("invalid", "입력값 오류", "Invalid input"),
    ("m_rise", "상승 시간 10–90 %", "Rise time 10–90 %"),
    ("m_os", "오버슈트", "Overshoot"),
    ("m_settle", "2 % 정착 시간", "2 % settling time"),
    ("m_ss", "정상상태 오차", "Steady-state error"),
    ("m_ipk", "최대 전류 / 한계", "Peak current / limit"),
    ("m_sat", "포화 시간", "Time saturated"),
    ("m_wout", "출력축 최대 속도", "Peak output speed"),
    ("m_wmot", "모터 최대 속도 / 한계", "Peak motor speed / limit"),
    ("m_jeff", "J_eff / Kτ = N·η·Kt", "J_eff / Kτ = N·η·Kt"),
    ("na", "없음", "n/a"),
    ("angle", "각도 (deg)", "Angle (deg)"),
    ("current", "전류 (A)", "Current (A)"),
];

fn tr(key: &str) -> &'static str {
    let ko = ST.with(|s| s.borrow().ko);
    TEXT.iter()
        .find(|t| t.0 == key)
        .map(|t| if ko { t.1 } else { t.2 })
        .unwrap_or("?")
}

fn doc() -> Document {
    web_sys::window().unwrap().document().unwrap()
}

fn input(id: &str) -> HtmlInputElement {
    doc().get_element_by_id(id).unwrap().dyn_into().unwrap()
}

fn num(id: &str) -> f64 {
    input(id).value().trim().parse().unwrap_or(f64::NAN)
}

fn set_num(id: &str, v: f64) {
    // Round to 6 significant digits, then print the shortest decimal form.
    let r: f64 = format!("{:.5e}", v).parse().unwrap_or(v);
    input(id).set_value(&format!("{}", r));
}

fn set_text(id: &str, s: &str) {
    if let Some(e) = doc().get_element_by_id(id) {
        e.set_text_content(Some(s));
    }
}

fn apply_lang() {
    let d = doc();
    let ko = ST.with(|s| s.borrow().ko);
    d.document_element().unwrap().set_attribute("lang", if ko { "ko" } else { "en" }).ok();
    let list = d.query_selector_all("[data-i18n]").unwrap();
    for k in 0..list.length() {
        let el: HtmlElement = list.item(k).unwrap().dyn_into().unwrap();
        if let Some(key) = el.dataset().get("i18n") {
            el.set_text_content(Some(tr(&key)));
        }
    }
    set_text("lang", if ko { "English" } else { "한국어" });
}

fn load_preset(p: &Plant) {
    let vals = [
        ("j_rotor", p.j_rotor), ("kt", p.kt), ("r", p.r), ("v", p.v_supply), ("imax", p.i_max),
        ("n", p.n), ("eta", p.eta), ("j_load", p.j_load), ("b", p.b), ("tau_c", p.tau_c),
        ("tau_g", p.tau_g), ("max_rpm", p.max_motor_rpm),
    ];
    for (id, v) in vals {
        set_num(id, v);
    }
    let c = Pid::default_for(p);
    for (id, v) in [("kp", c.kp), ("ki", c.ki), ("kd", c.kd), ("fc", c.d_cutoff_hz), ("fs", c.sample_hz)] {
        set_num(id, v);
    }
    input("enc").set_value("0");
    input("dmeas").set_checked(c.d_on_meas);
    let s = SimCfg::default();
    set_num("step", s.step_deg);
    set_num("dur", s.duration_s);
}

fn read_plant() -> Plant {
    Plant {
        j_rotor: num("j_rotor"), kt: num("kt"), r: num("r"), v_supply: num("v"), i_max: num("imax"),
        n: num("n"), eta: num("eta"), j_load: num("j_load"), b: num("b"), tau_c: num("tau_c"),
        tau_g: num("tau_g"), max_motor_rpm: num("max_rpm"),
    }
}

fn read_pid() -> Pid {
    let enc = num("enc");
    Pid {
        kp: num("kp"), ki: num("ki"), kd: num("kd"), d_cutoff_hz: num("fc"),
        d_on_meas: input("dmeas").checked(), sample_hz: num("fs"),
        enc_counts: if enc.is_finite() && enc >= 1.0 { enc.min(1e9) as u32 } else { 0 },
    }
}

fn fmt(v: Option<f64>, unit: &str, scale: f64, dec: usize) -> String {
    match v {
        Some(x) if x.is_finite() => format!("{:.*} {}", dec, x * scale, unit),
        _ => tr("na").to_string(),
    }
}

fn run() {
    let p = read_plant();
    let s = SimCfg { step_deg: num("step"), duration_s: num("dur") };
    let (bw, zeta) = (num("bw"), num("zeta"));
    let g = suggest_gains(&p, bw, zeta);
    set_text("sugg", &format!("Kp = {:.4}, Ki = {:.4}, Kd = {:.5}", g.kp, g.ki, g.kd));
    let tr_res = simulate(&p, &read_pid(), &s);
    let trace = match tr_res {
        Ok(t) => t,
        Err(field) => {
            set_text("verdict", &format!("{}: {}", tr("invalid"), field));
            ST.with(|st| st.borrow_mut().last = None);
            draw();
            return;
        }
    };
    let m = metrics(&trace, &p);
    let (v, reasons) = verdict(&m, &p);
    let head = tr(match v { Verdict::Ok => "ok", Verdict::Warn => "warn", Verdict::Bad => "bad" });
    let why: Vec<&str> = reasons
        .iter()
        .map(|r| tr(match r {
            Reason::Unstable => "r_unstable",
            Reason::MotorOverspeed => "r_overspeed",
            Reason::Overshoot => "r_overshoot",
            Reason::Saturated => "r_sat",
            Reason::NotSettled => "r_settle",
        }))
        .collect();
    let mut head = head.to_string();
    if !why.is_empty() {
        head.push_str(" — ");
        head.push_str(&why.join(", "));
    }
    set_text("verdict", &head);
    let rows = [
        ("m_rise", fmt(m.rise_s, "ms", 1e3, 1)),
        ("m_os", fmt(Some(m.overshoot_pct), "%", 1.0, 1)),
        ("m_settle", fmt(m.settle_s, "ms", 1e3, 1)),
        ("m_ss", fmt(Some(m.ss_err_deg), "deg", 1.0, 3)),
        ("m_ipk", format!("{:.2} / {:.2} A", m.peak_current, p.i_max)),
        ("m_sat", fmt(Some(m.sat_pct), "%", 1.0, 1)),
        ("m_wout", fmt(Some(m.peak_out_rpm), "rpm", 1.0, 1)),
        ("m_wmot", format!("{:.0} / {:.0} rpm", m.peak_motor_rpm, p.max_motor_rpm)),
        ("m_jeff", format!("{:.4e} kg·m² / {:.4} N·m/A", p.j_eff(), p.k_tau())),
    ];
    let html: String = rows
        .iter()
        .map(|(k, v)| format!("<tr><td>{}</td><td>{}</td></tr>", tr(k), v))
        .collect();
    doc().get_element_by_id("metrics").unwrap().set_inner_html(&html);
    ST.with(|st| st.borrow_mut().last = Some(trace));
    draw();
}

struct Palette {
    fg: &'static str,
    grid: &'static str,
    main: &'static str,
    ghost: &'static str,
    sp: &'static str,
    band: &'static str,
}

fn palette() -> Palette {
    let dark = web_sys::window()
        .and_then(|w| w.match_media("(prefers-color-scheme: dark)").ok().flatten())
        .map(|m| m.matches())
        .unwrap_or(false);
    if dark {
        Palette { fg: "#ccc", grid: "#353a42", main: "#5aa9ff", ghost: "#888", sp: "#ff9f43", band: "rgba(255,159,67,0.15)" }
    } else {
        Palette { fg: "#333", grid: "#e2e2e2", main: "#0a66c2", ghost: "#aaa", sp: "#d9480f", band: "rgba(217,72,15,0.10)" }
    }
}

/// Plot `lines` (t, y, color) with horizontal reference lines and an optional shaded band.
fn plot(id: &str, title: &str, lines: &[(&[f64], &[f64], &str)], hlines: &[f64], band: Option<(f64, f64)>) {
    let pal = palette();
    let cv: HtmlCanvasElement = doc().get_element_by_id(id).unwrap().dyn_into().unwrap();
    let dpr = web_sys::window().unwrap().device_pixel_ratio();
    let (w, h) = (cv.client_width() as f64, cv.client_height() as f64);
    cv.set_width((w * dpr) as u32);
    cv.set_height((h * dpr) as u32);
    let c: CanvasRenderingContext2d = cv.get_context("2d").unwrap().unwrap().dyn_into().unwrap();
    c.scale(dpr, dpr).ok();
    c.clear_rect(0.0, 0.0, w, h);
    let (l, r, t, b) = (52.0, 8.0, 18.0, 22.0);
    let t_max = lines.iter().filter_map(|s| s.0.last()).fold(1e-9f64, |a, &x| a.max(x));
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    let ys = lines.iter().flat_map(|s| s.1.iter().copied()).chain(hlines.iter().copied());
    for y in ys.filter(|y| y.is_finite()) {
        lo = lo.min(y);
        hi = hi.max(y);
    }
    if !lo.is_finite() {
        (lo, hi) = (-1.0, 1.0);
    }
    let pad = ((hi - lo) * 0.08).max(1e-6);
    let (lo, hi) = (lo - pad, hi + pad);
    let px = |x: f64| l + (x / t_max) * (w - l - r);
    let py = |y: f64| t + (hi - y) / (hi - lo) * (h - t - b);
    c.set_font("11px system-ui, sans-serif");
    c.set_fill_style_str(pal.fg);
    c.fill_text(title, l, 12.0).ok();
    c.set_line_width(1.0);
    c.set_stroke_style_str(pal.grid);
    for k in 0..=4 {
        let fy = lo + (hi - lo) * k as f64 / 4.0;
        let fx = t_max * k as f64 / 4.0;
        c.begin_path();
        c.move_to(l, py(fy));
        c.line_to(w - r, py(fy));
        c.move_to(px(fx), t);
        c.line_to(px(fx), h - b);
        c.stroke();
        c.fill_text(&format!("{:.3}", fy), 2.0, py(fy) + 4.0).ok();
        let dx = if k == 4 { 30.0 } else { 12.0 };
        c.fill_text(&format!("{:.2}s", fx), px(fx) - dx, h - 6.0).ok();
    }
    if let Some((a, z)) = band {
        c.set_fill_style_str(pal.band);
        c.fill_rect(l, py(z), w - l - r, py(a) - py(z));
    }
    c.set_stroke_style_str(pal.sp);
    c.set_line_dash(&js_sys_array(&[5.0, 4.0])).ok();
    for y in hlines {
        c.begin_path();
        c.move_to(l, py(*y));
        c.line_to(w - r, py(*y));
        c.stroke();
    }
    c.set_line_dash(&js_sys_array(&[])).ok();
    c.set_line_width(1.6);
    for (xs, ys, color) in lines {
        let color = match *color { "ghost" => pal.ghost, _ => pal.main };
        c.set_stroke_style_str(color);
        c.begin_path();
        // Decimate to about 2 points per pixel.
        let step = (xs.len() / (2 * w as usize).max(1)).max(1);
        for (k, (x, y)) in xs.iter().zip(ys.iter()).enumerate().step_by(step) {
            if k == 0 { c.move_to(px(*x), py(*y)) } else { c.line_to(px(*x), py(*y)) }
        }
        c.stroke();
    }
}

fn js_sys_array(v: &[f64]) -> JsValue {
    let a = js_sys::Array::new();
    for x in v {
        a.push(&JsValue::from_f64(*x));
    }
    a.into()
}

fn draw() {
    ST.with(|st| {
        let st = st.borrow();
        let Some(cur) = st.last.as_ref() else {
            plot("c_angle", tr("angle"), &[], &[], None);
            plot("c_current", tr("current"), &[], &[], None);
            return;
        };
        let sp = cur.setpoint_deg;
        let mut a: Vec<(&[f64], &[f64], &str)> = Vec::new();
        let mut i: Vec<(&[f64], &[f64], &str)> = Vec::new();
        if let Some(g) = st.ghost.as_ref() {
            a.push((&g.t, &g.theta_deg, "ghost"));
            i.push((&g.t, &g.current, "ghost"));
        }
        a.push((&cur.t, &cur.theta_deg, "main"));
        i.push((&cur.t, &cur.current, "main"));
        let band = (sp - 0.02 * sp.abs(), sp + 0.02 * sp.abs());
        plot("c_angle", tr("angle"), &a, &[sp], Some(band));
        plot("c_current", tr("current"), &i, &[cur.i_max, -cur.i_max], None);
    });
}

fn on(id: &str, ev: &str, f: impl FnMut() + 'static) {
    let cb = Closure::<dyn FnMut()>::new(f);
    doc().get_element_by_id(id).unwrap()
        .add_event_listener_with_callback(ev, cb.as_ref().unchecked_ref()).unwrap();
    cb.forget();
}

fn storage() -> Option<web_sys::Storage> {
    web_sys::window()?.local_storage().ok().flatten()
}

#[wasm_bindgen(start)]
pub fn start() {
    let win = web_sys::window().unwrap();
    let saved = storage().and_then(|s| s.get_item("pid-step-lang").ok().flatten());
    let ko = match saved.as_deref() {
        Some(l) => l == "ko",
        None => win.navigator().language().unwrap_or_default().starts_with("ko"),
    };
    ST.with(|s| s.borrow_mut().ko = ko);
    apply_lang();
    load_preset(&Plant::small_servo());

    on("form", "input", run);
    on("preset", "change", || {
        let v = doc().get_element_by_id("preset").unwrap().dyn_into::<web_sys::HtmlSelectElement>().unwrap().value();
        load_preset(&if v == "knee" { Plant::humanoid_knee() } else { Plant::small_servo() });
        run();
    });
    on("apply", "click", || {
        let g = suggest_gains(&read_plant(), num("bw"), num("zeta"));
        if g.kp.is_finite() && g.ki.is_finite() && g.kd.is_finite() {
            set_num("kp", g.kp);
            set_num("ki", g.ki);
            set_num("kd", g.kd);
        }
        run();
    });
    on("pin", "click", || {
        ST.with(|s| {
            let mut s = s.borrow_mut();
            s.ghost = s.last.clone();
        });
        draw();
    });
    on("unpin", "click", || {
        ST.with(|s| s.borrow_mut().ghost = None);
        draw();
    });
    on("lang", "click", || {
        let ko = ST.with(|s| {
            let mut s = s.borrow_mut();
            s.ko = !s.ko;
            s.ko
        });
        if let Some(s) = storage() {
            s.set_item("pid-step-lang", if ko { "ko" } else { "en" }).ok();
        }
        apply_lang();
        run();
    });
    let cb = Closure::<dyn FnMut()>::new(draw);
    win.add_event_listener_with_callback("resize", cb.as_ref().unchecked_ref()).unwrap();
    cb.forget();
    run();
}
