# Stage 4 report — turn controller for a given angle

> Sections map to checkpoint 4 (guide p.25). Design sections are pre-drafted
> with the actual implementation; measurement sections take numbers from
> MEASURE_STAGE4.md.

## 1. Kinematic derivation — wheel distance for a given angle [checkpoint]

Differential drive with effective track b (guide p.23): Δθ = (s_R − s_L)/b.

**Spin turn** (wheels counter-rotate, equal magnitudes): s_R = −s_L = s_w.
Δθ = (s_w − (−s_w))/b = 2·s_w/b → **s_w = b·Δθ/2** per wheel.
Example at b = 160 mm, Δθ = 90° = π/2: s_w = 160·π/4 = 126 mm ≈ 25 counts.

**Pivot turn** (one wheel planted): s_planted = 0, the other wheel s_m.
Δθ = s_m/b → **s_m = b·Δθ** — twice the spin per-wheel distance, all on one wheel.

Conclusion (resolution, sec. 2): one encoder count = k mm of wheel arc.
- spin: Δθ_1count = 2k/b — HALF the resolution of...
- pivot: Δθ_1count = k/b — the moving wheel measures the whole arc directly.

Both derivations in code: `wheelTargetMM()` (spin: b·rad/2; pivot: b·rad) and
`progressDeg()` (spin: (sR−sL)/b; pivot L: sR/b; pivot R: −sL/b).

## 2. Angular resolution of this robot

k = 5.1087 mm/count (stage 2), b from calibration (sec. 3 below).
- spin: ______ °/count; pivot: ______ °/count (half the spin value)
- counts per wheel: 15° spin ≈ ______ , 30° ≈ ______ , 45° ≈ ______ , 90° ≈ ______
- STOP_TOL_DEG = 0.7° ≈ 1.5 counts — smaller than the 15° turn's ______ counts,
  so the tolerance does not swallow a small turn, but it is the same order as
  the quantisation at 15°: the guide's "15° lies close to the resolution limit"
- the 45° requirement = ______ counts — comfortably above the limit ✓
- improvements considered (guide p.23 table): pivot mode for small angles
  (2× resolution — implemented via the `m` mode switch), lower turn speed
  (PWM_CRUISE=110 < straight cruise; run-on shrink), operator loop-closing
  (the `tturn` result line carries `got=` so phase-1 of stage 6 can correct).

## 3. Track calibration [checkpoint]

- tape measurement: ~160 mm (nominal estimate)
- multi-rev: L1440/R1440, protractor readings and correction factors →
  calibrated b = ______ mm (MEASURE_STAGE4 sec. 1)
- difference vs tape: ______ % — explained by: tyre contact-point shift under
  twist, lateral slip of the tyres in a spin turn, caster friction moment
  (the guide's three reasons); the calibrated value is the one in the formula
- L vs R correction difference: ______ → caster asymmetry noted / not observed

## 4. Controller design (implementation notes)

- **Speed profile as a function of progress** (analogous to stage 3): cruise
  PWM_CRUISE=110, angular ramps ACC_DEG/DEC_DEG=30°, floor PWM_MIN=85 (the
  floor-measured dead band + margin from stage 3 — turns use the same floor).
- **Stopping condition**: rem ≤ BRAKE_ALLOW_DEG + STOP_TOL_DEG. The DEC ramp
  guarantees the wheels enter the last 30° at the floor speed, so the brake
  run-on is a near-constant body angle (measured in sim: 2.5–3.2°) — a constant
  allowance beats a live speed estimate, which lags the deceleration and fires
  the stop early by a variable amount (first sim iteration failed exactly that
  way: 15° turns stopped at 11°).
- **Symmetry controller (spin only)**: proportional on the difference of wheel
  path MAGNITUDES, |cL|−|cR| — equalising magnitudes, not signed difference
  (guide p.24). Clamp ±CORR_MAX=20 applied to the magnitudes, never to the
  signed pwm: clamping the signed value would flip a backward-rotating wheel
  to forward and stall the turn (found in sim: 194-count magnitude spread).
- **Pivot mode has NO symmetry controller** — one wheel is planted, there is
  nothing to equalise (guide p.24 note, implemented as motorL(0)/motorR(0)).
- **Sign convention**: positive = left = ccw from above; L/R letters carry the
  direction; negative numbers take absolute value (same rule as stage 3 F/B).
  Recorded for stage 5's protocol description.
- **Result line**: `tturn cmd=L90.0 got=89.9deg L=-197cnt R=198cnt` — the
  achieved angle per odometry (`turnAchievedDeg()`) is in every response, as
  the guide requires for stage 6 phase-1 corrections.

## 5. Measurements (from MEASURE_STAGE4.md)

### 5.1 Track calibration
| | commanded | achieved | factor | b_used |
|---|---|---|---|---|
| L1440 | 1440° | | | |
| R1440 | 1440° | | | |
- calibrated track: ______ mm; tape: ~160 mm; difference explained (sec. 3)

### 5.2 Spin accuracy (mean ± spread over 5 reps)
| angle | dir | odometric mean | protractor mean | spread |
|---|---|---|---|---|
| 15° | L/R | | | |
| 30° | L/R | | | |
| 45° | L/R | | | |
| 90° | L/R | | | |

### 5.3 Pivot accuracy (small angles)
| angle | odometric mean | protractor mean | spread |
|---|---|---|---|
| 15° | | | |
| 30° | | | |

### 5.4 Diagnosis (guide p.24 table)
- mean error vs angle: ______ → conclusion: ______
- spread vs tuning: ______ → limitation is one of ______ (resolution / control)

### 5.5 Mode comparison at 15°
- spin spread ______ vs pivot spread ______ → pivot's 2× resolution ______
  (visible / not visible at 15°) — the strongest argument in the report

## 6. 45° requirement [checkpoint]

- worst-case |mean error| at 45°: ______° ≤ 5% of 45° = 2.25°? ☐ yes / ☐ no
- the assignment's "at least 45° resolution" is met: ☐

## 7. Command interface notes (feeding stage 5)

- `L<deg>` / `R<deg>` (T/B aliases), `m` cycles spin→pivotL→pivotR→spin,
  `J<mm>` track override for calibration, out-of-range 5..1440° rejected
- every turn ends with one result line containing the achieved angle
- unknown characters skipped; bare letter rejected after 300 ms (stage 3 rules)
