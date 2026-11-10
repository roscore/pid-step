# Task: Robot joint PID step-response simulator (Rust → WebAssembly, static web app)

Original open-source tool. Write all code yourself; do not copy anyone's code, tables or assets.
Use textbook control/motor equations only. Presets are generic, clearly labelled "typical" values that
users can edit (no vendor part numbers).

## Product (SMALL scope — keep the whole tool ~400-800 lines of Rust)
A single-page web app (GitHub Pages, sub-path `/pid-step/`): "how will my geared robot joint respond to a
position step with these PID gains — and does it saturate?"

1. **Plant (one geared joint)**: motor rotor inertia (kg·m²), motor torque constant Kt (N·m/A), winding
   resistance R (Ω), supply voltage (V), max motor current (A), gear ratio N, gear efficiency (0–1),
   load inertia at the output (kg·m²), viscous friction at output (N·m·s/rad), Coulomb friction at output
   (N·m), optional constant gravity torque at output (N·m). Model at the output shaft:
   J_eff = J_load + N²·J_rotor; torque command τ = N·η·Kt·i, current limited by max current AND by
   back-EMF voltage headroom ((V − Ke·ω_motor)/R, Ke = Kt in SI). Electrical inductance neglected (say so).
   Two presets: "small servo joint (typical)" and "humanoid knee joint (typical)" (editable).
2. **Controller**: discrete PID on output angle at a user sample rate (default 1 kHz): Kp, Ki, Kd,
   derivative low-pass filter cutoff (Hz), derivative on measurement (checkbox, default on), integrator
   clamp anti-windup, output = current command (A) saturated at the limit. Optional encoder resolution
   (counts/rev at output, 0 = ideal) quantizing the measured angle.
3. **Simulation + metrics**: step of user size (deg, default 30°) from 0, duration (default 1 s), fixed-step
   semi-implicit Euler with internal substeps (≥ 10 per controller period). Outputs: 10–90 % rise time,
   overshoot %, 2 % settling time, steady-state error (deg), peak current and time spent saturated (%),
   peak output speed (rpm), peak motor speed vs a user max motor rpm. Verdict ✅ ok / ⚠️ (overshoot > 10 %
   or saturated > 20 % of the time or not settled) / ❌ (unstable / diverging or motor speed over limit).
4. **Plot** (canvas, no libs): angle vs time with setpoint and ±2 % band; second small plot of current
   with the ± limit lines. Optional "pin current run" button that overlays the previous trace (ghost) for
   comparison — this is the only extra besides #5.
5. **Suggest gains** (extra): from desired closed-loop bandwidth ω_n (Hz) and damping ζ (default 0.9),
   compute PD gains by pole placement on the linearized plant (Kp = J_eff·ω_n²/Kτ,
   Kd = (2ζω_n·J_eff − b)/Kτ where Kτ = N·η·Kt), Ki = Kp·ω_n/10; one "apply" button.
6. Korean + English toggle (browser default). Mobile friendly. CSS prefers-color-scheme only.
Out of scope (do NOT build): Bode/Nyquist plots, multi-joint, inductance/current loop, URL share,
JSON import/export, auto-tuning search, 3D.

## Core crate (pure Rust, no web deps, unit tested) — `crates/ps_core`
- Plant, controller, simulator, metrics, verdict, gain suggestion.
- Tests (hand-checkable, 4–8): J_eff = J_load + N²·J_rotor; current saturation respected at every step;
  P-only with no friction/gravity settles to ~0 error; gravity load with Ki = 0 leaves steady-state error
  ≈ τ_g/(Kp·Kτ) and Ki > 0 removes it; metrics on a synthetic first-order trace (rise time ≈ 2.197·τ);
  suggested gains give a response with overshoot < 5 % for ζ = 0.9 (unsaturated small step);
  huge Kp with tiny sample rate flagged unstable.
- `crates/ps_web`: UI. Plain web-sys — keep the bundle small.

## Repo hygiene
- README (Korean first, then English): what it is, model and assumptions (rigid joint, no inductance,
  ideal gearbox apart from efficiency, no backlash/compliance), how to use, local build
  (`cargo test -p ps_core`, `trunk build --release`), MIT license, "estimates only — validate on hardware
  with safe limits".
- LICENSE: MIT, copyright "pid-step contributors". `.gitignore` for target/ dist/.
- `index.html` with a CSP meta that blocks external requests:
  `default-src 'self'; script-src 'self' 'wasm-unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; base-uri 'self'`.
  In `Trunk.toml` set `filehash = false` and
  `pattern_script = "<script type=\"module\" src=\"{base}boot.js\"></script>"` with an external
  `web/boot.js` (copied via `<link data-trunk rel="copy-file">`) that imports `./ps_web.js` and calls
  init, so no inline script is needed.
- `Trunk.toml`: `public_url = "/pid-step/"`.
- `.github/workflows/pages.yml`: on push to main + workflow_dispatch; checkout, rust stable + wasm32
  target, install trunk 0.22.1 binary release, `sh tools/privacy_check.sh`,
  `cargo test -p ps_core --release`, `trunk build --release`, configure-pages@v5,
  upload-pages-artifact@v3 (dist), deploy job with actions/deploy-pages@v4
  (permissions pages: write, id-token: write).
- `tools/privacy_check.sh` already exists; keep it and keep it clean.

## Done when
- `cargo test -p ps_core` green; `trunk build --release` succeeds; `sh tools/privacy_check.sh` clean.
- Commit everything with clear commits; open a PR. Do NOT create new GitHub repos or change repo settings.
