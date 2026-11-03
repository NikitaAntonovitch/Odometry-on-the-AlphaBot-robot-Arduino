# Stage 3 — fill-in templates for the 5 pending checkpoint items

Status: TEMPLATE ONLY — every value is `<<unmeasured>>` until the robot
session writes it. No number in this file is evidence. Items map to
REPORT_STAGE3_S5_ASSEMBLED.md §5.8 (checkpoint-3 rows marked PENDING) and
MEASURE_STAGE3.md. Constants in force: K_MM 5.1087, PWM_CRUISE 130,
PWM_MIN 85, dead band ≈ 75 floor-measured (working value incl. margin 55 in
stage 1; 85 is the profile floor used by stages 3–6), K_SYM 3.0, ACC_MM 60,
DEC_MM 60, STOP_TOL_MM 10.

Firmware: flash `lab1_stage3_straight.ino` (stage-3 sketch with the wheel
guard). Distance runs use `F<mm>` / `B<mm>`; symmetry gain is set at runtime
with `K<value>` (no reflash); `e` prints encoder counts; `t`-style telemetry
in stage 3 is via the `s`/speed diagnostics — see §d for the telemetry build.

Pre-flight (same as every block): encoder hand-push `z` → push 0.5 m → `e`
(both wheels count, R/L 0.9…1.1); pack charged (3 start-stalls = sagged pack,
abort the block); note voltage if a multimeter is at hand: ______ V.

Abort rule (all sections): an aborted run (`err:` line) is EVIDENCE — write
ABORT in the row, copy the err line, continue. Do not delete or re-run in
place.

## a. Ramp behaviour — jerk / overshoot / run-on (replaces the "+50 mm informal run-on" note)

> **Procedure:** mark a start line and a finish line at the target distance
> (tape). Send the command, watch the start for jerk/slip chirp, then measure
> where the wheel axis actually stopped relative to the finish line:
> positive = past the mark (overshoot/run-on), negative = short. One row =
> one command. Run F200 and F500 twice each; add B500 rows for direction
> symmetry.

| run | direction | target_mm | command sent | overshoot_mm (tape, +past/−short) | start jerk (none/visible) | notes |
|---|---|---|---|---|---|---|
| 1 | fwd | 200 | F200 | <<unmeasured>> | <<unmeasured>> | |
| 2 | fwd | 200 | F200 | <<unmeasured>> | <<unmeasured>> | |
| 3 | fwd | 500 | F500 | <<unmeasured>> | <<unmeasured>> | |
| 4 | fwd | 500 | F500 | <<unmeasured>> | <<unmeasured>> | |
| 5 | back | 500 | B500 | <<unmeasured>> | <<unmeasured>> | |

Decision this feeds: if |overshoot| at F500 > ~15 mm (run-on confirmed at
arrival speed), DEC_MM 60 grows and/or STOP_TOL tightens — record the
DEC_MM/STOP_TOL decision: `<<unmeasured>>` (no change / adjusted to ___/___).

## b. K6 — the 4th gain value (saturation demonstration)

> **Procedure:** `B500` runs at each gain; read the L−R end-count difference
> off the done line (1 count = 5.109 mm). Existing rows are REAL data from
> the 14.10 session (REPORT_STAGE3_S5_ASSEMBLED §5.2, copied verbatim);
> fill ONLY the K6 rows. Expected symptom at K6 (PRED on file): corr exceeds
> CORR_MAX 30 for err > 5 mm → weaving / clamp bang-bang, not stall.

| gain (PWM/mm) | run1_lat_mm | run2_lat_mm | run3_lat_mm | run4_lat_mm | run5_lat_mm | mean | spread |
|---|---|---|---|---|---|---|---|
| 0 (reference) | 71.5 | 78.7 | — | — | — | 75.1 (n=2, REAL 14.10) | 5.1 |
| 1.0 | 20.4 | — | — | — | — | 20.4 (n=1, REAL 14.10) | — |
| 3.0 (CHOSEN) | 10.2 | — | — | — | — | 10.2 (n=1, REAL 14.10) | — |
| 6.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

K6 runs: send `K6` (confirm line), then 3–5 × `B500`; per row record
`L−R end cnt × 5.1087` in the *_lat_mm cells and the symptom (weaving?
stutter? none) here: ______. Checkpoint satisfied when the symptom line is
recorded, whatever it shows.

## c. Lateral deviation at F500 — K0 vs K3 (5 runs each)

> **Procedure:** straight lane ≥ 0.6 m, tape line on the floor. Start with the
> wheel axis on the mark. Run `F500`, measure the lateral deviation of the
> wheel-axis midpoint from the line at the stop (perpendicular, ±: left of
> intended direction positive). K0 first (send `K0`), then `K3`. This is the
> guide's "lateral deviation for zero gain and for the chosen gain" item.

K0 (reference — no correction):

| run | lateral_deviation_mm | notes |
|---|---|---|
| 1 | <<unmeasured>> | |
| 2 | <<unmeasured>> | |
| 3 | <<unmeasured>> | |
| 4 | <<unmeasured>> | |
| 5 | <<unmeasured>> | |

K3 (chosen gain):

| run | lateral_deviation_mm | notes |
|---|---|---|
| 1 | <<unmeasured>> | |
| 2 | <<unmeasured>> | |
| 3 | <<unmeasured>> | |
| 4 | <<unmeasured>> | |
| 5 | <<unmeasured>> | |

Mean/spread per gain (compute after filling): K0 mean ______ σ ______;
K3 mean ______ σ ______. Acceptance: K3 mean clearly below K0 mean, and the
improvement exceeds K0's spread (the same test that validated K1 at 14.10).

## d. Symmetry error vs time — two gains (telemetry captures)

> **Procedure:** the plain stage-3 printout only gives end-state numbers; the
> time series needs the telemetry build. Rebuild the stage-3 sketch with
> `-DTELEMETRY_TURN=1` (host sim equivalent: the `t` telemetry path), or use
> the `s` window-speed key repeatedly if a reflash is not convenient — the
> stream format below works for either. Capture with the serial monitor's
> copy-to-file (or a terminal logger) — do not transcribe by hand.

Steps:
1. Flash/build with telemetry enabled (do NOT change any control constant).
2. `K3` → `F500`, capture the stream to `stage3_tel_K3.log`.
3. `K6` → `F500`, capture to `stage3_tel_K6.log`.
4. Leave telemetry ON only for these runs — it competes with the control
   loop for loop time (guide's warning); disable for acceptance runs.

Plot stub — reads either log format (lines with `t=<ms> err=<mm>` OR plain
`<ms>,<mm>` pairs), plots both gains on one axes, saves PNG:

```python
# stage3_tel_plot.py - e(t) for two gains on the same axes (stdlib + matplotlib)
import re, sys
import matplotlib.pyplot as plt

def load(path):
    t, e = [], []
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.search(r"t=(\d+)\s+err=(-?\d+\.?\d*)", line)
        if m:
            t.append(int(m.group(1)) / 1000.0); e.append(float(m.group(2))); continue
        m = re.match(r"\s*(\d+)\s*[,;]\s*(-?\d+\.?\d*)\s*$", line)
        if m:
            t.append(int(m.group(1)) / 1000.0); e.append(float(m.group(2)))
    return t, e

series = [(g, load(p)) for g, p in zip(sys.argv[1::2], sys.argv[2::2])]
for (label, (t, e)) in series:
    plt.plot(t, e, label=label)
plt.xlabel("time (s)"); plt.ylabel("symmetry error e(t) (mm)")
plt.title("stage 3: wheel-difference error vs time, two gains")
plt.legend(); plt.grid(True); plt.savefig("stage3_tel.png", dpi=150)
print("saved stage3_tel.png")
# usage: python stage3_tel_plot.py K3 stage3_tel_K3.log K6 stage3_tel_K6.log
```

> **Session 30.10.2026 — TWO G0.5 telemetry runs, BOTH at the default gain
> (K_SYM 3.0): this VIOLATES the "two gains" requirement — the second-gain
> capture is still `<<unmeasured>>`. What the two same-gain runs DO show:
> oscillation amplitude TRIPLED between runs (run 1 peak-to-peak ±10.2 mm →
> run 2 ±15.3 mm; run 2 start already asymmetric pwmL=85/pwmR=102 at the
> first control step). Same firmware, same gain, same commanded distance →
> the growth points at environmental/state drift between runs (battery
> voltage, floor texture, operator start placement), NOT at a gain change.
> Flagged in the report; final done line still in tolerance
> (got=493.0, err=−7.0, L=96 R=97). Verbatim streams live in the session
> log; the capture files named below are still unfilled.

Captured files: K3 log ______ (rows: `<<unmeasured>>`), K6 log ______
(rows: `<<unmeasured>>` — the 30.10 runs above were BOTH at the default
gain, so they do not satisfy the two-gain requirement; a genuine K6 (or
K0) telemetry capture is still owed). Checkpoint satisfied when both PNGs exist and the
K6 trace visibly shows the clamp behaviour (flat-topped corr, weaving) or
its absence — either outcome is a documented result.

## e. Controller error vs scale error — odometry AND tape per run

> **Procedure (the guide's three numbers per run):** for each run record
> (1) the commanded distance, (2) the distance the robot's own odometry
> reports (`got=` on the done line), (3) the ACTUAL wheel-axis displacement
> measured with a tape between start and stop marks. Both measurements are
> mandatory per run — controller error without tape truth cannot separate
> the two error classes. Distances: F200, F500, B500, ×2 runs each.

Formulas (apply per run; the table keeps raw numbers only):
- controller_error = odometry − commanded   (did the robot reach its own target?)
- scale_error      = tape − odometry        (is k right? >±8.5 mm systematic on
  500 mm ⇒ k correction candidate — the ±2-count window is ±10.2 mm)

| run | command | commanded_mm | odometry_mm (got=) | tape_mm | controller_err (odo−cmd) | scale_err (tape−odo) |
|---|---|---|---|---|---|---|
| 1 | F200 | 200 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 2 | F200 | 200 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 3 | F500 | 500 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 4 | F500 | 500 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 5 | B500 | −500 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 6 | B500 | −500 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

Decision stub (compute after filling; do NOT edit K_MM on the floor):

```
mean_scale_err = mean(scale_err over runs)            # mm
if abs(mean_scale_err) > 8.5:      # ~2 counts on 500 mm
    k_new = K_MM * 500 / (500 + mean_scale_err)       # first-order correction
    verdict = "k correction CANDIDATE: K_MM -> <k_new> (needs its own confirm session)"
else:
    verdict = "k stands: scale error within the 2-count window"
dominant = "controller" if mean(|controller_err|) > mean(|scale_err|) else "scale"
```

Result: verdict = `<<unmeasured>>` — goes into REPORT_STAGE3 §5.7 and the
calibration audit note.
