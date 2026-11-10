# pid-step

로봇 관절 PID 스텝 응답 시뮬레이터 / Robot joint PID step-response simulator.

> **추정치일 뿐입니다 — 실제 하드웨어에서 안전한 제한값으로 검증하세요.**
> **Estimates only — validate on hardware with safe limits.**

## 한국어

### 무엇인가
"이 PID 게인으로 감속기 달린 로봇 관절에 위치 스텝을 주면 어떻게 움직일까, 그리고 포화될까?"에
답하는 단일 페이지 웹 앱입니다. Rust → WebAssembly로 빌드되며 브라우저 안에서만 동작합니다
(서버, 분석 도구, 외부 요청 없음. 언어 설정만 localStorage에 저장).

### 모델과 가정
- 출력축 기준 강체 관절: `J_eff·dω/dt = N·η·Kt·i − b·ω − τ_c·sgn(ω) − τ_g`,
  `J_eff = J_load + N²·J_rotor`.
- 전류는 최대 전류와 역기전력 전압 여유 `(V − Ke·ω_motor)/R` (SI 단위에서 `Ke = Kt`)로 제한.
- **전기 인덕턴스 무시** (전류가 지령을 즉시 추종), 효율 η 외에는 이상적인 감속기,
  **백래시·컴플라이언스 없음**, 중력 토크는 일정값.
- 이산 PID (출력 = 전류 지령, A): D 저역통과 필터, 측정값 미분(기본), 적분기 클램프 안티와인드업
  (|Ki·∫e| ≤ I_max), 선택적 엔코더 양자화.
- 고정 스텝 semi-implicit Euler, 제어 주기당 10개 이상의 내부 서브스텝.
- 게인 제안: 선형화 플랜트에 극배치. `Kp = J_eff·ω_n²/Kτ`, `Kd = (2ζω_n·J_eff − b)/Kτ`,
  `Ki = Kp·ω_n/10`, `Kτ = N·η·Kt`. 적분항 때문에 PD만일 때보다 오버슈트가 커집니다
  (ζ = 0.9에서 약 10–15 %).
- 프리셋 두 개("소형 서보 관절", "휴머노이드 무릎 관절")는 편집 가능한 **일반적인 예시 값**일
  뿐이며 특정 제품과 무관합니다.

### 사용법
파라미터를 바꾸면 즉시 다시 계산됩니다. 각도 그래프(목표값, ±2 % 대역)와 전류 그래프(± 한계선),
상승/정착 시간, 오버슈트, 정상상태 오차, 최대 전류와 포화 시간, 출력축/모터 최대 속도,
판정(✅ / ⚠️ / ❌)을 보여 줍니다. "현재 결과 고정"으로 이전 결과를 회색으로 겹쳐 비교할 수 있습니다.

**3D 보기**: 모터·감속기·출력 암을 3D로 그려 스텝 응답을 재생합니다(기본 0.1배속). 드래그로 시점을
돌리고, 슬라이더로 원하는 시각을 고를 수 있습니다. 그래프에는 같은 시각이 세로선으로 표시됩니다.
전류·전압이 포화된 순간에는 모터가 빨간색이 되고, 고정한 이전 결과는 회색 암으로 겹쳐 보입니다.
주황 막대는 로터 각도(출력각 × N)입니다. WebGL 없이 canvas 2D 원근 투영만 씁니다.

### 로컬 빌드
```sh
rustup target add wasm32-unknown-unknown
cargo test -p ps_core
trunk build --release      # dist/ 생성 (public_url = /pid-step/)
sh tools/privacy_check.sh
```

### 라이선스
MIT — `LICENSE` 참조.

## English

### What it is
A single-page web app answering "how will my geared robot joint respond to a position step with these
PID gains — and does it saturate?". Rust compiled to WebAssembly, runs entirely in the browser
(no server, no analytics, no network requests; only the language choice is kept in localStorage).

### Model and assumptions
- Rigid joint at the output shaft: `J_eff·dω/dt = N·η·Kt·i − b·ω − τ_c·sgn(ω) − τ_g`,
  `J_eff = J_load + N²·J_rotor`.
- Current limited by the max current and by back-EMF voltage headroom `(V − Ke·ω_motor)/R`
  (`Ke = Kt` in SI units).
- **Electrical inductance neglected** (current follows the command instantly), ideal gearbox apart
  from efficiency η, **no backlash or compliance**, constant gravity torque.
- Discrete PID (output = current command, A): derivative low-pass filter, derivative on measurement
  (default), integrator clamp anti-windup (|Ki·∫e| ≤ I_max), optional encoder quantization.
- Fixed-step semi-implicit Euler with ≥ 10 internal substeps per controller period.
- Gain suggestion by pole placement on the linearized plant: `Kp = J_eff·ω_n²/Kτ`,
  `Kd = (2ζω_n·J_eff − b)/Kτ`, `Ki = Kp·ω_n/10`, `Kτ = N·η·Kt`. The integral term adds overshoot
  compared with the PD pair alone (about 10–15 % at ζ = 0.9).
- The two presets ("small servo joint", "humanoid knee joint") are generic **typical** values to edit,
  not any specific product.

### How to use
Edit any field and the run updates instantly: angle plot (setpoint, ±2 % band), current plot (± limit
lines), rise/settling time, overshoot, steady-state error, peak current and time saturated, peak output
and motor speed, and a verdict (✅ / ⚠️ / ❌). "Pin current run" overlays the previous trace in grey.

**3D view**: replays the step on a 3D motor + gearbox + output arm (0.1× by default). Drag to orbit,
scrub with the slider (the plots show the same instant as a vertical cursor). The motor turns red while
current/voltage is saturated, a pinned run appears as a grey arm, and the orange bar is the rotor angle
(output × N). Plain canvas 2D perspective, no WebGL.

### Local build
```sh
rustup target add wasm32-unknown-unknown
cargo test -p ps_core
trunk build --release      # writes dist/ (public_url = /pid-step/)
sh tools/privacy_check.sh
```

### License
MIT — see `LICENSE`.
