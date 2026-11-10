//! Core of pid-step: one geared robot joint, a discrete PID position loop,
//! a fixed-step simulator, step-response metrics, a verdict and a gain suggestion.
//!
//! Model (output shaft): J_eff·dω/dt = N·η·Kt·i − b·ω − τ_c·sgn(ω) − τ_g,
//! J_eff = J_load + N²·J_rotor. Rigid joint, no backlash/compliance, ideal gearbox apart
//! from efficiency η, electrical inductance neglected (current follows the command instantly
//! inside the current and back-EMF voltage limits).

use std::f64::consts::PI;

/// Physical joint parameters (SI units, output side unless noted).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plant {
    /// Motor rotor inertia, kg·m².
    pub j_rotor: f64,
    /// Motor torque constant, N·m/A (also back-EMF constant Ke in V·s/rad).
    pub kt: f64,
    /// Winding resistance, Ω.
    pub r: f64,
    /// Supply voltage, V.
    pub v_supply: f64,
    /// Max motor current, A.
    pub i_max: f64,
    /// Gear ratio (motor turns per output turn).
    pub n: f64,
    /// Gear efficiency, 0..1.
    pub eta: f64,
    /// Load inertia at the output, kg·m².
    pub j_load: f64,
    /// Viscous friction at the output, N·m·s/rad.
    pub b: f64,
    /// Coulomb friction at the output, N·m.
    pub tau_c: f64,
    /// Constant gravity torque at the output (resists positive motion), N·m.
    pub tau_g: f64,
    /// Max allowed motor speed, rpm (only used by the verdict).
    pub max_motor_rpm: f64,
}

impl Plant {
    /// Generic "small servo joint (typical)" values.
    pub fn small_servo() -> Self {
        Plant {
            j_rotor: 1.0e-6,
            kt: 0.012,
            r: 1.5,
            v_supply: 12.0,
            i_max: 3.0,
            n: 100.0,
            eta: 0.7,
            j_load: 0.003,
            b: 0.01,
            tau_c: 0.05,
            tau_g: 0.3,
            max_motor_rpm: 8000.0,
        }
    }

    /// Generic "humanoid knee joint (typical)" values.
    pub fn humanoid_knee() -> Self {
        Plant {
            j_rotor: 1.2e-4,
            kt: 0.1,
            r: 0.2,
            v_supply: 48.0,
            i_max: 40.0,
            n: 30.0,
            eta: 0.85,
            j_load: 0.15,
            b: 0.5,
            tau_c: 1.5,
            tau_g: 20.0,
            max_motor_rpm: 4000.0,
        }
    }

    /// Effective inertia reflected to the output shaft.
    pub fn j_eff(&self) -> f64 {
        self.j_load + self.n * self.n * self.j_rotor
    }

    /// Output torque per amp, N·m/A.
    pub fn k_tau(&self) -> f64 {
        self.n * self.eta * self.kt
    }

    /// Limit a current command by the current limit and the back-EMF voltage headroom
    /// at output speed `w_out` (rad/s). NaN-safe (NaN maps to 0).
    pub fn limit_current(&self, cmd: f64, w_out: f64) -> f64 {
        let emf = self.kt * self.n * w_out;
        let hi = (self.v_supply - emf) / self.r;
        let lo = (-self.v_supply - emf) / self.r;
        let cmd = if cmd.is_nan() { 0.0 } else { cmd };
        cmd.max(lo).min(hi).max(-self.i_max).min(self.i_max)
    }

    fn validate(&self) -> Result<(), &'static str> {
        let pos = [
            (self.j_rotor, "J_rotor"),
            (self.kt, "Kt"),
            (self.r, "R"),
            (self.v_supply, "V"),
            (self.i_max, "I_max"),
            (self.n, "N"),
            (self.eta, "eta"),
            (self.j_eff(), "J_eff"),
        ];
        for (v, name) in pos {
            if !(v.is_finite() && v > 0.0) {
                return Err(name);
            }
        }
        let nonneg = [(self.j_load, "J_load"), (self.b, "b"), (self.tau_c, "tau_c")];
        for (v, name) in nonneg {
            if !(v.is_finite() && v >= 0.0) {
                return Err(name);
            }
        }
        if self.eta > 1.0 {
            return Err("eta");
        }
        if !self.tau_g.is_finite() {
            return Err("tau_g");
        }
        Ok(())
    }
}

/// Discrete PID on output angle; output is a current command in A.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pid {
    /// A/rad
    pub kp: f64,
    /// A/(rad·s)
    pub ki: f64,
    /// A·s/rad
    pub kd: f64,
    /// Derivative low-pass cutoff, Hz (0 = no filter).
    pub d_cutoff_hz: f64,
    /// Differentiate the measurement instead of the error (no derivative kick).
    pub d_on_meas: bool,
    /// Controller sample rate, Hz.
    pub sample_hz: f64,
    /// Encoder counts per output revolution (0 = ideal measurement).
    pub enc_counts: u32,
}

impl Pid {
    /// Suggested gains for a plant at 5 Hz / ζ = 0.9, with typical filter settings.
    pub fn default_for(p: &Plant) -> Self {
        let g = suggest_gains(p, 5.0, 0.9);
        Pid {
            kp: g.kp,
            ki: g.ki,
            kd: g.kd,
            d_cutoff_hz: 100.0,
            d_on_meas: true,
            sample_hz: 1000.0,
            enc_counts: 0,
        }
    }
}

/// Step test settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimCfg {
    pub step_deg: f64,
    pub duration_s: f64,
}

impl Default for SimCfg {
    fn default() -> Self {
        SimCfg { step_deg: 30.0, duration_s: 1.0 }
    }
}

/// Max number of controller ticks per run (keeps the UI responsive).
pub const MAX_TICKS: usize = 200_000;

/// One recorded sample per controller tick.
#[derive(Clone, Debug, Default)]
pub struct Trace {
    pub t: Vec<f64>,
    /// True output angle, deg.
    pub theta_deg: Vec<f64>,
    /// Applied motor current at the start of the tick, A.
    pub current: Vec<f64>,
    /// Command was clipped (current or voltage limit) during the tick.
    pub saturated: Vec<bool>,
    pub setpoint_deg: f64,
    pub i_max: f64,
    /// Largest |applied current| over all substeps, A.
    pub peak_current: f64,
    /// Largest |output speed| over all substeps, rad/s.
    pub peak_w_out: f64,
    /// Run stopped early because the angle ran away.
    pub diverged: bool,
}

/// Simulate a position step from 0 using semi-implicit Euler with internal substeps.
pub fn simulate(p: &Plant, c: &Pid, s: &SimCfg) -> Result<Trace, &'static str> {
    p.validate()?;
    for (v, name) in [(c.kp, "Kp"), (c.ki, "Ki"), (c.kd, "Kd"), (c.d_cutoff_hz, "fc")] {
        if !(v.is_finite() && v >= 0.0) {
            return Err(name);
        }
    }
    if !(c.sample_hz.is_finite() && c.sample_hz > 0.0) {
        return Err("fs");
    }
    if !(s.step_deg.is_finite() && s.step_deg != 0.0) {
        return Err("step");
    }
    if !(s.duration_s.is_finite() && s.duration_s > 0.0) {
        return Err("duration");
    }
    let ticks = (s.duration_s * c.sample_hz).round();
    if !(1.0..=MAX_TICKS as f64).contains(&ticks) {
        return Err("duration×fs");
    }
    let ticks = ticks as usize;
    let ts = 1.0 / c.sample_hz;
    // >= 10 substeps per period, finer for slow controllers, bounded total work.
    let sub = ((ts / 1e-4).ceil() as usize).clamp(10, (4_000_000 / ticks).max(10));
    let h = ts / sub as f64;

    let sp = s.step_deg.to_radians();
    let j = p.j_eff();
    let kt = p.k_tau();
    let q = if c.enc_counts > 0 { 2.0 * PI / c.enc_counts as f64 } else { 0.0 };
    let alpha = if c.d_cutoff_hz > 0.0 {
        let tau = 1.0 / (2.0 * PI * c.d_cutoff_hz);
        ts / (tau + ts)
    } else {
        1.0
    };
    let i_lim = if c.ki > 0.0 { p.i_max / c.ki } else { 0.0 };

    let mut tr = Trace {
        setpoint_deg: s.step_deg,
        i_max: p.i_max,
        ..Default::default()
    };
    let (mut th, mut w) = (0.0f64, 0.0f64);
    let (mut integ, mut d_f, mut prev_meas, mut prev_err) = (0.0, 0.0, 0.0, 0.0);

    for k in 0..ticks {
        let meas = if q > 0.0 { (th / q).round() * q } else { th };
        let e = sp - meas;
        let raw_d = if c.d_on_meas { -(meas - prev_meas) / ts } else { (e - prev_err) / ts };
        prev_meas = meas;
        prev_err = e;
        d_f += alpha * (raw_d - d_f);
        // Integrator clamp anti-windup: |Ki·∫e| never exceeds the current limit.
        integ = (integ + e * ts).max(-i_lim).min(i_lim);
        let u = c.kp * e + c.ki * integ + c.kd * d_f;
        let cmd = if u.is_nan() { 0.0 } else { u.max(-p.i_max).min(p.i_max) };
        let mut sat = cmd != u;

        let theta0 = th;
        let mut i_first = 0.0;
        for m in 0..sub {
            let i = p.limit_current(cmd, w);
            if m == 0 {
                i_first = i;
            }
            sat |= i != cmd;
            tr.peak_current = tr.peak_current.max(i.abs());
            let drive = kt * i - p.b * w - p.tau_g;
            if w == 0.0 && drive.abs() <= p.tau_c {
                // Stuck by static (Coulomb) friction.
            } else {
                let dir = if w != 0.0 { w.signum() } else { drive.signum() };
                let w_new = w + h * (drive - p.tau_c * dir) / j;
                // Friction cannot reverse the motion: stop at zero crossing.
                w = if w != 0.0 && w_new.signum() != w.signum() { 0.0 } else { w_new };
            }
            th += h * w;
            tr.peak_w_out = tr.peak_w_out.max(w.abs());
        }
        tr.t.push(k as f64 * ts);
        tr.theta_deg.push(theta0.to_degrees());
        tr.current.push(i_first);
        tr.saturated.push(sat);
        if !th.is_finite() || (th - sp).abs() > 10.0 * sp.abs() {
            tr.diverged = true;
            break;
        }
    }
    Ok(tr)
}

/// Step-response numbers.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Metrics {
    /// 10–90 % rise time, s.
    pub rise_s: Option<f64>,
    pub overshoot_pct: f64,
    /// 2 % settling time, s (None = not settled within the run).
    pub settle_s: Option<f64>,
    /// Setpoint − final angle (mean of last 5 %), deg.
    pub ss_err_deg: f64,
    pub peak_current: f64,
    pub sat_pct: f64,
    pub peak_out_rpm: f64,
    pub peak_motor_rpm: f64,
    pub unstable: bool,
}

fn crossing(t: &[f64], y: &[f64], level: f64) -> Option<f64> {
    let k = y.iter().position(|&v| v >= level)?;
    if k == 0 {
        return Some(t[0]);
    }
    let (y0, y1) = (y[k - 1], y[k]);
    Some(t[k - 1] + (t[k] - t[k - 1]) * (level - y0) / (y1 - y0))
}

/// Rise, overshoot, settling and steady-state error of a trace `y` (any unit) towards `sp`.
pub fn step_metrics(t: &[f64], y: &[f64], sp: f64) -> (Option<f64>, f64, Option<f64>, f64) {
    if t.is_empty() || t.len() != y.len() || sp == 0.0 {
        return (None, 0.0, None, f64::NAN);
    }
    let yn: Vec<f64> = y.iter().map(|v| v / sp).collect();
    let rise = match (crossing(t, &yn, 0.1), crossing(t, &yn, 0.9)) {
        (Some(a), Some(b)) => Some(b - a),
        _ => None,
    };
    let peak = yn.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let overshoot = ((peak - 1.0) * 100.0).max(0.0);
    let settle = match yn.iter().rposition(|v| v.is_nan() || (v - 1.0).abs() > 0.02) {
        None => Some(t[0]),
        Some(k) if k + 1 < t.len() => Some(t[k + 1]),
        _ => None,
    };
    let n = (y.len() / 20).max(1);
    let tail = &y[y.len() - n..];
    let ss = sp - tail.iter().sum::<f64>() / n as f64;
    (rise, overshoot, settle, ss)
}

fn max_abs(v: &[f64]) -> f64 {
    v.iter().fold(0.0, |a, x| if x.abs() > a { x.abs() } else { a })
}

/// Metrics for a simulated trace.
pub fn metrics(tr: &Trace, p: &Plant) -> Metrics {
    let (rise_s, overshoot_pct, settle_s, ss_err_deg) =
        step_metrics(&tr.t, &tr.theta_deg, tr.setpoint_deg);
    let n = tr.t.len();
    let sat = tr.saturated.iter().filter(|&&s| s).count();
    let err: Vec<f64> = tr.theta_deg.iter().map(|v| v - tr.setpoint_deg).collect();
    // Growing error late in the run = diverging oscillation.
    let (mid, late) = (max_abs(&err[n / 4..n / 2]), max_abs(&err[3 * n / 4..]));
    let growing = n >= 8 && late > 1.05 * mid && late > 0.05 * tr.setpoint_deg.abs();
    let rpm = 60.0 / (2.0 * PI);
    Metrics {
        rise_s,
        overshoot_pct,
        settle_s,
        ss_err_deg,
        peak_current: tr.peak_current,
        sat_pct: if n > 0 { 100.0 * sat as f64 / n as f64 } else { 0.0 },
        peak_out_rpm: tr.peak_w_out * rpm,
        peak_motor_rpm: tr.peak_w_out * p.n * rpm,
        unstable: tr.diverged || growing || !ss_err_deg.is_finite(),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Verdict {
    Ok,
    Warn,
    Bad,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reason {
    Unstable,
    MotorOverspeed,
    Overshoot,
    Saturated,
    NotSettled,
}

/// ❌ unstable / overspeed, ⚠️ overshoot > 10 % / saturated > 20 % / not settled, else ✅.
pub fn verdict(m: &Metrics, p: &Plant) -> (Verdict, Vec<Reason>) {
    let mut bad = Vec::new();
    if m.unstable {
        bad.push(Reason::Unstable);
    }
    if m.peak_motor_rpm > p.max_motor_rpm {
        bad.push(Reason::MotorOverspeed);
    }
    if !bad.is_empty() {
        return (Verdict::Bad, bad);
    }
    let mut warn = Vec::new();
    if m.overshoot_pct > 10.0 {
        warn.push(Reason::Overshoot);
    }
    if m.sat_pct > 20.0 {
        warn.push(Reason::Saturated);
    }
    if m.settle_s.is_none() {
        warn.push(Reason::NotSettled);
    }
    let v = if warn.is_empty() { Verdict::Ok } else { Verdict::Warn };
    (v, warn)
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Gains {
    pub kp: f64,
    pub ki: f64,
    pub kd: f64,
}

/// Pole placement on the linearized plant J_eff·s² + b·s with gain Kτ:
/// Kp = J·ω_n²/Kτ, Kd = (2ζω_n·J − b)/Kτ (≥ 0), Ki = Kp·ω_n/10. `bw_hz` is ω_n in Hz.
pub fn suggest_gains(p: &Plant, bw_hz: f64, zeta: f64) -> Gains {
    let wn = 2.0 * PI * bw_hz;
    let (j, k) = (p.j_eff(), p.k_tau());
    let kp = j * wn * wn / k;
    Gains {
        kp,
        ki: kp * wn / 10.0,
        kd: ((2.0 * zeta * wn * j - p.b) / k).max(0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Frictionless, gravity-free plant with generous electrical limits.
    fn linear_plant() -> Plant {
        Plant { b: 0.0, tau_c: 0.0, tau_g: 0.0, i_max: 1e6, v_supply: 1e6, ..Plant::small_servo() }
    }

    fn pid(kp: f64, ki: f64, kd: f64) -> Pid {
        Pid { kp, ki, kd, ..Pid::default_for(&Plant::small_servo()) }
    }

    fn run(p: &Plant, c: &Pid, step: f64, dur: f64) -> (Trace, Metrics) {
        let tr = simulate(p, c, &SimCfg { step_deg: step, duration_s: dur }).unwrap();
        let m = metrics(&tr, p);
        (tr, m)
    }

    #[test]
    fn j_eff_reflects_rotor_through_gear() {
        let p = Plant { j_load: 0.5, j_rotor: 2e-4, n: 50.0, ..Plant::small_servo() };
        assert!((p.j_eff() - (0.5 + 2500.0 * 2e-4)).abs() < 1e-12);
    }

    #[test]
    fn current_saturation_respected_every_step() {
        let p = Plant::small_servo();
        let (tr, m) = run(&p, &pid(500.0, 50.0, 5.0), 90.0, 1.0);
        assert!(tr.current.iter().all(|i| i.abs() <= p.i_max + 1e-12));
        assert!(m.peak_current <= p.i_max + 1e-12);
        assert!(m.sat_pct > 0.0);
    }

    #[test]
    fn p_only_without_coulomb_or_gravity_settles_to_zero_error() {
        // Viscous friction only (no Coulomb, no gravity): P alone has no steady-state error.
        let p = Plant { b: 0.2, ..linear_plant() };
        let (_, m) = run(&p, &pid(2.0, 0.0, 0.0), 10.0, 5.0);
        assert!(!m.unstable);
        assert!(m.ss_err_deg.abs() < 0.01, "ss = {}", m.ss_err_deg);
    }

    #[test]
    fn gravity_droop_matches_theory_and_integral_removes_it() {
        let p = Plant { tau_g: 0.5, b: 0.05, ..linear_plant() };
        let (kp, kd) = (40.0, 1.0);
        let (_, m) = run(&p, &pid(kp, 0.0, kd), 10.0, 3.0);
        let expect = (p.tau_g / (kp * p.k_tau())).to_degrees();
        assert!((m.ss_err_deg - expect).abs() < 0.01 * expect, "{} vs {}", m.ss_err_deg, expect);
        let (_, m) = run(&p, &pid(kp, 200.0, kd), 10.0, 5.0);
        assert!(m.ss_err_deg.abs() < 0.01, "ss with Ki = {}", m.ss_err_deg);
    }

    #[test]
    fn metrics_of_first_order_trace() {
        let tau = 0.1;
        let t: Vec<f64> = (0..20_000).map(|k| k as f64 * 1e-4).collect();
        let y: Vec<f64> = t.iter().map(|t| 30.0 * (1.0 - (-t / tau).exp())).collect();
        let (rise, os, settle, ss) = step_metrics(&t, &y, 30.0);
        assert!((rise.unwrap() - 2.197 * tau).abs() < 1e-3);
        assert!(os < 1e-9);
        assert!((settle.unwrap() - 3.912 * tau).abs() < 1e-3); // ln(50)·τ
        assert!(ss.abs() < 0.01);
    }

    #[test]
    fn suggested_pd_gains_overshoot_below_5pct() {
        let p = Plant { tau_c: 0.0, tau_g: 0.0, ..Plant::small_servo() };
        let g = suggest_gains(&p, 5.0, 0.9);
        // Unsaturated small step with the pole-placed PD pair.
        let (_, m) = run(&p, &pid(g.kp, 0.0, g.kd), 2.0, 1.0);
        assert_eq!(m.sat_pct, 0.0);
        assert!(m.overshoot_pct < 5.0, "overshoot {}", m.overshoot_pct);
        assert_eq!(verdict(&m, &p).0, Verdict::Ok);
    }

    #[test]
    fn huge_kp_at_low_sample_rate_is_unstable() {
        let p = linear_plant();
        let c = Pid { sample_hz: 20.0, ..pid(1e4, 0.0, 0.0) };
        let (_, m) = run(&p, &c, 30.0, 1.0);
        assert!(m.unstable);
        assert_eq!(verdict(&m, &p).0, Verdict::Bad);
    }

    #[test]
    fn suggested_full_pid_is_stable_and_removes_error() {
        // Ki = Kp·ω_n/10 makes the loop type 2, so it overshoots more than the PD pair
        // (≈ 14 % here); it must still be stable, settle and null the error.
        let p = Plant { tau_c: 0.0, tau_g: 0.0, ..Plant::small_servo() };
        let g = suggest_gains(&p, 5.0, 0.9);
        let (_, m) = run(&p, &pid(g.kp, g.ki, g.kd), 2.0, 2.0);
        assert!(!m.unstable && m.settle_s.is_some());
        assert!(m.ss_err_deg.abs() < 0.01, "ss = {}", m.ss_err_deg);
    }
}
