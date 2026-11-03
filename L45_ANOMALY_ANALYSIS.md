# L45 anomaly — isolation-battery analysis (session 30.10.2026)

Status: DESK ANALYSIS ONLY — no firmware modified, no data invented.
Source data: stage-4 isolation battery, spin mode, one rep per angle
(stage4_accuracy_template.md rows marked "Session data 30.10.2026").

## The numbers

| run | got= | N_L | N_R | coast | sum ticks | asym \|L\|−\|R\| |
|---|---|---|---|---|---|---|
| L90 | +89.2° | −22 | 24 | 2 | 46 | −2 |
| **L45** | **+40.7°** | **−4** | **17** | **1** | **21** | **−13** |
| L15 | +15.5° | −3 | 5 | 2 | 8 | −2 |
| R90 | −89.2° | 26 | −20 | 2 | 46 | +6 |
| R45 | <<not yet run>> | — | — | — | — | — |

Internal consistency (sanity, all pass): got = (|N_L|+|N_R|)/2 × 3.877°/pair
for every row (L45: 10.5 pairs → 40.71° vs printed 40.7). The L45 counters
are not a logging glitch — the wheel pair genuinely turned 40.7° with the
work split 4/17 instead of ~11.6/11.6 (nominal 11.6 ticks/wheel at
b=151, k=5.1087). R did +5.4 ticks past nominal; L did −7.6 short.

## H1 — ramp dominates short turns: REFUTED as the primary cause

Firmware: `ACC_DEG = 30.0f` / `DEC_DEG = 30.0f` (stage-7 .ino:70–71), applied
to every turn regardless of angle (:839–844: `uAcc = prog / ACC_DEG`,
`uDec = rem / DEC_DEG`, vDeg blended to the PWM_MIN floor while `u < 1`).

Arithmetic: 30° of body-angle ramp = 7.7 ticks/wheel. L45's whole commanded
wheel motion is 11.6 ticks ≈ 1.5× one ramp — the guide's warning DOES apply
(least correction headroom inside the ramp). But it applies EQUALLY to L15
(3.9 ticks, 100 % inside ramps, |asym| = 2) and L90 (cruise 30→36.5° only,
|asym| = 2). A pure ramp effect would scale with the fraction of motion spent
ramping — worst for L15 — and would not produce a 4:1 split with the SAME
sign convention as a healthy run. The ramp explains why short turns are
VULNERABLE (see H2), not the 13-count imbalance itself.

## H2 — one wheel below breakaway at start, correction too weak to free it: CONFIRMED as the consistent mechanism

Firmware chain:

- `turnStart` (:703–724) fires ONE breakaway pulse `T_PULSE` at
  `PULSE_PWM = 120` for `PULSE_MS = 150` (:80–81), then goes `T_RUN`. If the
  wheel breaks free there, fine — L90/L15 did.
- In `T_RUN`, the spin symmetry controller (:853–864):
  `err = (|cL| − |cR|)·K`, `corr = symGain·err` clamped to
  `CORR_MAX_TRN = 20` (:79), `magL = pwm − corr; magR = pwm + corr`, then
  `if (magL < PWM_MIN) magL = PWM_MIN;` (PWM_MIN = 85, :47).
- If L stalls at start: err = (4−17)·5.1087 ≈ −66 mm → corr saturates at
  −20 → magL = pwm+20, magR = pwm−20. With base pwm ≈ 85–100 (turns run at
  PWM_CRUISE_TRN = 110 with a PWM_MIN = 85 floor), the boost reaches only
  ~105–120 — at or below the measured ~120 stiction breakaway — while magR
  is pinned at the 85 floor and keeps turning. The controller is HELPING,
  just not enough: max boost (+20 pwm) < breakaway deficit. That is the
  stage-7 stall pattern at motion start.
- **Stage-7 fault detection is NOT active during turns — by design, and this
  is the gap.** `faultArm()` is called only in `beginDrive()` (:377, straight
  drives); `turnStart` (:703–724) never arms it; the ratio/quiet detectors
  (:500–522) live in `controlStep()`, which runs only while `mot == M_RUN`
  (loop dispatch :1174–1177: turns run `turnStep()` instead). `turnStep()`
  has no faultArmed / STALL_WIN_MS / faultEnterHold reference (verified by
  source scan). During the L45 run the fault layer was disarmed, so nothing
  could catch L's stall — the design covers commanded G-drives only
  (REPORT_STAGE7 §1.3: "detectors live only inside a commanded drive").

## H3 — sign/magnitude-branch error in the correction: REFUTED

The correction is one straight-line formula with no magnitude branch:
`corr = (int)(symGain * err + (err >= 0 ? 0.5f : -0.5f))` then two clamps
(:858–861). The only direction dependence is the final mirror
`if (tsign > 0) … else …` (:864–865), which is symmetric and proven by R90
(|asym| = 6, healthy) and the stage-5 turn pool (~45 rows where
got = (cR−cL)·k/b exact). No `if (target < threshold) disable corr` exists.
L90 passing kills any sign-flip theory; the L45 failure mode is magnitude-
dependent only in the physical sense that 45° is the smallest angle whose
11.6-tick budget outlives the single 150 ms breakaway pulse while remaining
too big to finish inside the pulse (L15 = 3.9 ticks finishes almost entirely
inside the pulse+ramp with modest asymmetry of 2 ticks).

## Cross-check against history: L45 has ALWAYS been the collapse angle

REPORT_STAGE5 §2 turn table: L45 n=16, mean got ≈ 36.2°, E −8.8°, σ ≈ 6,
"collapse-contaminated"; R45 n=9, mean −40.9°, E +4.1°. Today's L45
(got 40.7, E −4.3) sits INSIDE the historic L45 distribution. This is not a
new regression — it is the known per-start collapse (limitation f of the
frozen stage-5/6 firmware) showing its statistics at n=1, on the same
magnitude that already triggered the R45 collapse in the 23.10 five-command
chain (E −17.9°).

## Verdict

**H2 confirmed (high confidence), with H1 as the enabling condition**
(short turn = small tick budget = one stall event dominates the outcome).
The turn controller lacks (a) the repeated breakaway retry the straight
controller effectively has via its longer travel, and (b) any stall
detection — the stage-7 layer is straight-drive-only.

## Recommendation: (c) STOP and investigate before more robot time

Justification: the Stage-6 acceptance route contains R45, the same magnitude
as L45. The 23.10 chain already showed R45 collapsing (E −17.9°) and today
L45 shows the same family of failure. Running the full 5-rep tables now
would spend ~30 of the remaining 90 floor-minutes documenting a defect
whose fix (turn-side stall detection or per-wheel breakaway pulses — the
designed-but-unapplied stage-4 patch) is already predicted to move σ 10.7→2.0
and E −10.3→−4.1. The 90/1440 cells are healthy and can wait a session; the
45° cells decide the assignment's "45° accuracy" gate and the acceptance
run.

**Diagnostics before re-running (user has telemetry willingness confirmed
for the next session):**

1. Desk: flash stage-7 sketch rebuilt with `-DTELEMETRY_TURN=1` (comment at
   :867–868 documents the hook; constants untouched). No firmware-logic edit.
2. Robot: `z` → `t` (telemetry on) → `L45` → capture `tturn_tel tgt_ticks=…
   NL= NR= pwmL= pwmR= e= corr=` lines. Expected signature if H2 holds:
   `NL` frozen at 0–2 for the first ~1–2 s while `pwmL` shows the corr-boosted
   value and `corr` pinned at ±20.
3. Same for `R45` (the acceptance-relevant direction).
4. Optional discriminator between "L stalls at start" vs "L stalls
   intermittently": run `L45` twice more with telemetry; compare whether
   `NL` moves at all in the first 500 ms.
