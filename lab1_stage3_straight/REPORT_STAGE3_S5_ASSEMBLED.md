## 5. Measurements — assembled skeleton (robot session DEFERRED)

> **State of this section:** design sections 1–4 above are final; this section
> is the measurement skeleton with every real number already in place and every
> missing number visibly marked. Labels: **REAL** = recorded in a past session
> (14.10 tuning / stage-2 calibration / code structure), **PRED** = model
> prediction with its source cited, **【TODO】/ TODO** = not measured (session
> deferred). No checkpoint item below is closed on prediction.

### 5.1 Ramp behaviour

Design intent (sec. 1): ACC_MM = 60 kills launch slip, DEC_MM = 60 shrinks
run-on to a brake-sized stop, STOP_TOL_MM = 10 (2 counts, the stage-2
quantisation floor).

- start jerk: 【TODO: measured value】 (informal observation only: launch reads
  smooth; no formal log)
- overshoot at F500: **observed informally, quantification pending** — the
  robot ends ≈ 4–6 cm past the finish on an unlogged run. Diagnosis from code
  review: `got=` prints at the stop decision, and by code structure (sec. 5.5)
  got= ∈ [490, 495] for every F500 — so the extra ≈ 50 mm CANNOT be an odometry
  error; it is **run-on after `brake()` from the ≈ 92 PWM arrival speed**
  (V_FLOOR = 85 ≈ 197 mm/s commanded). NOT a k error. The tape measurement
  that would quantify it is deferred (run-on block below).
- DEC_MM/STOP_TOL_MM adjusted: ☐ no / ☐ yes, to ___ / ___ — 【TODO: measured
  value; decision depends on the run-on quantification】

**Open design flag (recorded, not resolved):** DEC_MM = 60 was sized on the
claim "run-on almost removed at V_FLOOR". The informal +50 mm observation
suggests otherwise. DEC_MM stays 60 until the run-on block (5.6) is measured;
if run-on confirms ≈ 50 mm at arrival speed, DEC_MM grows and/or STOP_TOL
tightens — that re-tune is part of the deferred session, not this skeleton.

### 5.2 Gain tuning (Table 1) — REAL data, 14.10 session

> Method: gain changed at runtime via `K<value>` (no reflash); B500 runs;
> L−R end-count difference read from the done-line counters. Conversions:
> 1 count of difference = 5.1087 mm = 1.94° heading (track 151 mm, provisional).
> Gains are PWM per mm of wheel-path difference.

**Table 1 — L−R end difference vs symmetry gain (B500)**

| gain (PWM/mm) | n | L−R end mean (cnt) | spread (cnt) | L−R mean (mm) | heading equiv (°) | symptom observed | verdict |
|---|---|---|---|---|---|---|---|
| 0 (reference) | 2 | 14.0 | 8.0 | 71.5 | 27.1 | visible rightward curve at cruise; no correction fighting the +7 % asymmetry | baseline |
| 1.0 | 1 | 4.0 | 0 (n=1) | 20.4 | 7.8 | curve visibly reduced; no stutter, no weaving | effective |
| 3.0 — **CHOSEN** | 1 | 2.0 | 0 (n=1) | 10.2 | 3.9 | no visible symptoms: straight path, normal stop | best measured equalisation → K_SYM = 3.0f |
| 6.0 | TODO | TODO (not measured) | TODO (not measured) | TODO (not measured) | TODO (not measured) | **PRED:** corr exceeds CORR_MAX = 30 for err > 5 mm (30 ÷ 6) → expect weaving / clamp bang-bang, not wheel stall (per-wheel floor makes sub-dead-band commands impossible) | pending robot session |

*Footnote: n < 3 where noted — session scope (two K0 runs to establish the
baseline, one run per working gain for the first pass). The K0→K1 improvement
(14→4) exceeds the K0 spread (8), so it is a real controller effect; the
K1→K3 step (4→2) is within single-run resolution and is treated as indicative.*

**Model cross-check (PRED, equilibrium model: steady L−R ≈ 9 PWM ÷ (K·5.109)):**
K1 → 1.76 cnt (measured 4, residual +2.24); K3 → 0.59 cnt (measured 2, residual
+1.41). **Hypothesis, labeled as such:** residuals come from the start
transient (correction needs travel to build), asymmetry near the dead band /
in the ramps exceeding the +7 % cruise value, and reduced correction authority
while a wheel sits at the PWM floor. Distinguishing them = telemetry (Table 3).

**Chosen: K_SYM = 3.0** — smallest measured residual (≈ 10 mm path difference
≈ 3.9° heading equivalent), zero observed symptoms, correction demand at the
largest seen disturbance (≈ 31 PWM) just reaching CORR_MAX: the clamp is the
safety net, not an active limiter. K1 = documented fallback.

### 5.3 Lateral deviation, K = 0 vs chosen (Table 2) — ALL measurement cells TODO

> Protocol (deferred): 5× F500 per configuration, robot re-placed on the same
> mark, perpendicular distance from the start line's parallel measured at the
> stop. Sign convention: **+ = robot's left**. The per-run counter cross-check
> column catches runs where the L−R end difference drifted from the Table 1
> values (different floor state → different asymmetry).

**Table 2 — lateral deviation at 500 mm**

| run | K0 offset (mm, + = left) | K0 L−R cross-check (cnt) | K3 offset (mm, + = left) | K3 L−R cross-check (cnt) |
|---|---|---|---|---|
| 1 | TODO | TODO | TODO | TODO |
| 2 | TODO | TODO | TODO | TODO |
| 3 | TODO | TODO | TODO | TODO |
| 4 | TODO | TODO | TODO | TODO |
| 5 | TODO | TODO | TODO | TODO |
| **mean** | **TODO** | — | **TODO** | — |

**PRED anchors (arc model y = R(1−cos Δθ), R = path/Δθ, from the REAL Table 1
counts; kept in the final table only as a comparison column):**

| config | Δθ from REAL counts | y PRED | x-projection PRED |
|---|---|---|---|
| K0, 10 cnt | 19.4° | 84 mm | 491 mm |
| K0, 18 cnt | 34.9° | 148 mm | 470 mm |
| K3, 2 cnt | 3.9° | 17 mm | 500 mm |

Note the anchor's second prediction: at K0 the curved path eats 10–30 mm of
forward projection (x lands ≈ 470–490 mm, not 500) — if the taped x-positions
confirm this, it also slightly perturbs the Table 4 scale-error reading at K0.
The improvement claim the checkpoint wants ("controller improves accuracy")
will rest on the TODO means, not on these anchors.

### 5.4 Symmetry error vs time, two gains (Table 3) — TODO

> Protocol (deferred): `t` telemetry on, F500 at K3 and K6, tel lines captured
> to files; the chosen gain should pull the error to its steady state early,
> K6 should show the clamp band.

**Table 3 — symmetry error vs time (telemetry)**

| config | steady-state err PRED | measured steady-state err | time/ travel to settle | clamp activity observed |
|---|---|---|---|---|
| K3 | **PRED** ≈ 1.5–3 mm at cruise (equilibrium 0.59 cnt = 3.0 mm; ramp portion lower-authority) | TODO | TODO | TODO |
| K6 | **PRED** clamp band ±5 mm of err (corr saturates beyond err = 5 mm; expect bang-bang between clamp and floor) | TODO | TODO | TODO |

The K3 PRED row doubles as the resolution of the Table 1 hypothesis: if the
telemetry shows the error settling near 3 mm after the ramp, the residual is
the equilibrium prediction working and the Table 1 "measured 2 cnt" was the
same quantity quantised; a settling well above 3 mm supports the
dead-band/asymmetry hypothesis instead.

### 5.4a Model predictions for the symmetry-gain sweep (PRED)

> **Everything in this section is PRED** — model output from REAL-calibrated
> constants, provided because the K0/K3 sweep is postponed (pack discharged,
> session row 9). Measured column: **TODO pending battery recharge**. No PRED
> value enters session.csv or any measurement table cell.

**Constants used (all REAL-calibrated):** k = 5.1087 mm/cnt; cruise 302 mm/s
@ pwm 130; PWM_MIN 85; dead band ≈ 75; CORR_MAX 30; loop tick ~10 ms
(~3.0 mm/tick at cruise); stop rem grid {0.9, 3.9, 6.9, 9.9} mm; DEC_MM 60,
decel pwm = 85 + 45·(rem/60); coast run-on +7.7 mm (measured, row 2);
v5 start-kick 200 ms @ 140 (patch pending).

**Table P1 — parameter-free predictions (F500, any K):** every value PRED
from code structure + calibrated constants.

| quantity | PRED |
|---|---|
| got= | 490–499 mm |
| err | ∈ [−10, −1] mm, tick-quantised |
| settled run-on | +8 mm (measured anchor +7.7, row 2) |
| drive duration | ~2.0 s |
| start pwm sequence | 85 → kick 140 → 130 |
| per-wheel mean counts | ~97–98 |

**Table P2 — parametric predictions.** Model: raw wheel split g = right/left
speed ratio; d(err)/ds = −(g−1) along the path; equilibrium correction
c\* = 55·(g−1)/(g+1) PWM; steady-state residual = c\*/K. Every value PRED.

| g | K0 end diff | K3 residual | K6 residual | K0 pwm split | K3 pwm split | K6 pwm split |
|---|---|---|---|---|---|---|
| 1.05 | −25 mm | −0.4 mm | −0.2 mm | 130/130 flat | ±1–3 | ±2–6 |
| 1.10 | −50 mm | −0.9 mm | −0.4 mm | 130/130 flat | ±1–3 | ±2–6 |
| 1.15 | −75 mm | −1.3 mm | −0.6 mm | 130/130 flat | ±1–3 | ±2–6 |

(End diff at K0 = full uncorrected asymmetry; K3/K6 residuals sit at
equilibrium; pwm split is the mid-run corr signature "if corr healthy".)

**Anchor paragraph:** "Measured K6 F500 (session row 8) gives L-R = -15.3 mm,
25-70x above prediction. A working K6 controller cannot rest at -15 mm (equilibrium
would require corr = 92 PWM, above the 30 clamp). Two open hypotheses: (a) corr not
applied - signature: pwmL/pwmR = 130/130 flat throughout telemetry; (b) residual
left-encoder bias ~3% - signature: corr active (+/-2-6 PWM) while counts still end
near -3 counts. The telemetry K sweep decides between them."

### 5.5 Controller error vs scale error (Table 4) — decomposition + TODO cells

> The guide's point (p.21): commanded vs odometry ("did the robot reach its
> own target") and odometry vs tape ("is k right") are DIFFERENT errors, often
> confused. Separating them needs tape truth, which is deferred.

**Code-structural fact (REAL, by construction, holds for every future run):**
the stop fires when remaining ≤ 10 mm, and one 20 ms control step at stop
speed covers ≈ 3.9–4.3 mm (V_FLOOR = 85 ≈ 197 mm/s commanded). Therefore
**got= ∈ [cmd−10, cmd−6] mm for F; [cmd+6, cmd+10] for B** — i.e. F500 always
reports got= ∈ [490, 495]. This is the controller's designed window, not a
measurement.

**Table 4 — controller vs scale error, F/B series**

| run | commanded | got= (odometry) 【TODO: measured value】 | tape truth 【TODO: measured value】 | err_ctl = got − cmd | err_scale = tape − got | run-on after stop 【TODO: measured value】 |
|---|---|---|---|---|---|---|
| F500 | +500 mm | TODO | TODO | **PRED** ∈ [−10, −6] mm (code structure) | TODO | TODO |
| F200 | +200 mm | TODO | TODO | **PRED** ∈ [−10, −6] mm | TODO | TODO |
| B500 | −500 mm | TODO | TODO | **PRED** ∈ [+6, +10] mm (sign flips — the window is symmetric about the target, documented) | TODO | TODO |

**Three-way decomposition logic (the analysis the data will feed):**

1. **err_ctl** is known in advance within [−10, −6] F / [+6, +10] B — if a
   measured got= lands outside its window, that is a bug finding (stop logic
   or counter), not noise.
2. **run-on** is the tape-vs-got gap attributable to motion after the stop
   decision: the informal +50 mm observation is the current unquantified
   candidate; it resolves into this column once taped.
3. **err_scale** is what remains after 1 and 2: it tests the k calibration.
   Consistency bound: k spread 1.54 % ⇒ ±7.7 mm over 500 mm (±3.1 mm over
   200 mm). A scale error within ±8.5 mm on F500 = k confirmed, no correction;
   beyond it, k gets corrected (K_MM update) and this table records the old
   and new values.

**Status sentence (required):** the informal ≈ +50 mm past-finish observation
is **resolved into run-on under review, pending tape truth** — the code-
structure argument (got= window) excludes odometry error as its cause, but
the magnitude is unverified; the deferred session's F500 tape runs close it.

### 5.6 Run-on quantification (stage-1 debt, feed into 5.1)

- coast-vs-brake run-on at PWM 130 arrival speed: TODO (not measured; session
  deferred). Method ready: stop-mark tape + the stage-3 sketch's `c`/`b` keys.
- DEC_MM = 60 sizing claim ("run-on almost removed at V_FLOOR") is on the
  table for re-inspection — see 5.1 flag.

### 5.7 Limitations

**Session incident (unresolved): In the first v4 hardware session a systematic odometry
split appeared: R/L = 1.56-1.82 in every run, both on the floor and with wheels off the
ground (airborne), excluding floor effects (traction, caster skid). Two candidates remain:
(a) left motor slower at equal PWM; (b) left encoder undercounting. A decisive hand-push
test (motors off, robot pushed straight ~0.5 m, encoders counted) was defined but not yet
executed. Historical evidence - Stage 4 calibration residual +6%, Stage 3 tape-vs-odometry
agreement within +/-8.5 mm - shows both encoders were healthy in earlier sessions, so the
fault arose between sessions or during handling. Consequences: (1) rows tagged FAULT:L/R
are excluded from aggregate tables; (2) symmetry-gain effectiveness is unresolved - the K6
run showed no measurable correction (R/L 1.57 vs 1.56 without control), pending a
telemetry re-run; (3) tape measurements are postponed, all tape cells remain TODO;
(4) the distance-control machinery itself validated independently of the fault: stop
window got=490.4 mm in [490,495] (err -9.6 mm), deceleration profile pwm 123 @ rem 50.4
and 106 @ rem 27.4 matching 85+45*(rem/60), coast run-on reduced to +7.7 mm.**

**Status update (same session): After connector reseating, two K6 runs (F300 got=296.3 mm,
R/L=1.15; F500 got=493.0 mm, R/L=1.03) show near-equal wheel odometry, consistent with
restoration of the left encoder. Formal confirmation (hand-push before/after pair) and
the telemetry controller-alive check remain outstanding. The battery pack discharged late
in the session (three start-stall aborts at PWM_MIN=85, session row 9), postponing the
K0/K3 sweep and tape metrology; model predictions for the pending sweep are provided in
the PRED section.**

The robot measurement session is **deferred**; this skeleton carries real data
from past sessions only. Exactly what remains: **K6 ×2 runs** (Table 1 row +
symptom); **K0/K3 ×3 taped F500 runs** (Tables 2 and 4, one session serves
both); **telemetry captures K3/K6** (Table 3); **e-after-settle counter reads
on the K3 runs** (Table 2 cross-check column); **coast-vs-brake run-on at
PWM 130** (5.6). The capture sheet and analyzer are ready: the session.csv
format exists (tools side), so pasting the raw log is the only manual step.

### 5.8 Checkpoint 3 status, item by item

| checkpoint item | status | evidence / missing run |
|---|---|---|
| Driving a given distance works in both directions, several distances | **SATISFIED (code + early-run evidence)** | F100/B100 sanity + B500 tuning runs + F/B series in sim (21/21); distances 100–500 exercised |
| The ramp works — no jerk at start, no overshoot beyond tolerance | **PENDING** | informal observation only; needs the 5.1 log + the run-on quantification (5.6) |
| Correction-versus-dead-band limit solved; decision justified | **SATISFIED (code + design evidence)** | per-wheel PWM_MIN floor + CORR_MAX ⇒ sub-dead-band command impossible; documented in §5.2 design note; early runs show no stall |
| Tuning documented: ≥ 4 gain values with symptoms | **PENDING** | 3 real gains recorded (K0/K1/K3); missing: K6 ×2 runs + symptom line (Table 1 last row) |
| Lateral deviation measured for zero gain and for the chosen gain | **PENDING** | missing: 5× F500 taped at K0 and K3 (Table 2); PRED anchors in place, not evidence |
| Symmetry error recorded against time for two gain values | **PENDING** | missing: telemetry captures K3 + K6 (Table 3) |
| Controller error and scale error separated; k corrected | **PENDING** | missing: tape truth for F200/F500/B500 (Table 4); err_ctl window and the ±8.5 mm k-bound are prepared, decomposition logic written |

### 5.9 When data arrives — rework checklist

1. Paste the raw serial log into the session folder; fill the session.csv
   (format already defined) from it.
2. Recompute means/spreads with the analyzer (it emits the table rows in
   report format).
3. Replace every 【TODO: measured value】 slot in 5.1–5.6; keep the label of
   each former PRED that the data confirmed or refuted (one word: confirmed /
   refuted + the measured value).
4. Delete the PRED columns from Tables 2–4 and move them (with the Table 1
   equilibrium cross-check) to an appendix — final tables carry measured
   values only.
5. Re-run the 5.8 checkpoint marking with the new evidence; only then flip
   PENDING → SATISFIED, never before.
