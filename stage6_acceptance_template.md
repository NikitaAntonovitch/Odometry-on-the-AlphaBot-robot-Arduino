# Stage 6 — acceptance A→B template (fill on the floor)

Status: TEMPLATE ONLY — every value is `<<unmeasured>>` until the robot
session writes it. No number in this file is evidence. This is the GRADING
GATE: the guide's single acceptance criterion (operator drives A→B, robot is
carried back to A, one string drives it back to B autonomously, ×3).

Firmware: `lab1_stage6_autonomous.ino` stage6-v1-FROZEN (fault layer lives in
stage 7's sketch; the acceptance is on the frozen source, byte-identical
logic proven by the stage-7 report).

## Phase 1 — route (RECORDED, prior session — do not re-derive)

Recorded command string (one line, sent as a single paste):

    G0.8 L90 G1.2 R45 G0.6

Legs: A → 0.8 m straight → 90° left spin → 1.2 m straight → 45° right spin
→ 0.6 m straight → B. Predicted end in the odometry frame: (1224, 1624) mm
from A (computed 23.10; note row 4 collapse there produced ≈17 cm miss —
the acceptance measures whether the FROZEN firmware lands in the B zone).

Orientation mark at A (reproduce EXACTLY — the guide's warning is quantified:
a 5° alignment error at A puts B off by ~10 cm):

- [ ] wheel axis centre over point A (tape cross on the floor)
- [ ] heading along the A→first-leg direction (tape line or straightedge)
- [ ] B marked: tape cross at the predicted end point, B-zone = ±15 cm circle
      (acceptance gate used in REPORT_STAGE6 §3 decision rule)

Known risk carried from 23.10: R45 per-start collapse (limitation f,
E −17.9° observed once). If a collapse happens in phase 2: RECORD it, do not
re-run in place; the run-card gate then triggers the breakaway patch path.

## Phase 2 — three autonomous runs of the same string

> **Command per run:** place the robot on A + orientation mark, send the
> whole string `G0.8 L90 G1.2 R45 G0.6` as ONE line, touch nothing until the
> fifth result line, then measure. Carry back to A between runs (not reset
> by `z` — physical placement, per the assignment).

| run | start_pos (A) | end_pos_x_mm | end_pos_y_mm | heading_err_deg | commands_executed | aborts |
|---|---|---|---|---|---|---|
| 1 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 2 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 3 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

Column definitions:

- **start_pos (A):** `on mark` or a short note of the deviation from the mark
  (mm / degrees) — an honest note here explains phase-2 outliers later.
- **end_pos_x_mm:** signed offset of the wheel-axis midpoint from the B cross,
  x = along the nominal B direction (positive = overshot past B).
- **end_pos_y_mm:** signed offset, y = to the robot's left of the B direction.
- **heading_err_deg:** final heading vs the nominal B heading, read on the
  floor mark (CCW +, CW −), ±0.5° paper protractor.
- **commands_executed:** count of `go`+result line pairs seen (expect 5, all
  lines in order); write `4/5 + silent drop` etc. if fewer — that is a queue/
  parser event, evidence, not noise.
- **aborts:** number of `err:` abort lines during the run; copy each err line
  verbatim under the table (do not paraphrase).

Raw console per run (paste verbatim, nothing between go and the last line
edited):

```
run 1:
<paste>

run 2:
<paste>

run 3:
<paste>
```

Odometry cross-check per run (from the done/tturn lines, computed later):
odometric route length = Σ got of the three G legs. Tape the actual path
length with a tape along the floor marks: ______ mm (once, after run 3).

## Phase 3 — B-spread and error components

Computed from phase-2 data via `tools/stage6_process.py` — leave formulas,
do not hand-compute:

| field | value | note |
|---|---|---|
| mean shift along-route mx | computed from phase 2 data via stage6_process.py | systematic component → k (mmPerTick) |
| mean shift lateral my | computed from phase 2 data via stage6_process.py | systematic component → wheel track |
| spread σx, σy | computed from phase 2 data via stage6_process.py | random component — resolution + slip, tuning will not help |
| mean |deviation| from B | computed from phase 2 data via stage6_process.py | the headline number |
| systematic vs random verdict | computed from phase 2 data via stage6_process.py | mean > 2σ ⇒ systematic |
| odometry vs tape route length | computed from phase 2 data via stage6_process.py | scale/calibration error, NOT controller error |

Run it with the three (x, y) pairs + odometry and tape totals:

    python tools/stage6_process.py --x1 <x> --y1 <y> --x2 <x> --y2 <y> \
        --x3 <x> --y3 <y> --odo <odo_mm> --tape <tape_mm> \
        --label "stage6 acceptance G0.8 L90 G1.2 R45 G0.6"

Paste the summary block here: ______

## Decision gate (from the acceptance run card, REPORT_STAGE6 §3)

- 3/3 runs end in the B zone AND no turn off ≥ 4.7° → FREEZE stands
  (patch unused), phase 3 fills the spread.
- Any collapse run (turn off ≥ 4.7° or out of zone) → breakaway patch,
  re-run, freeze regardless. Verdict: ______

## Abort / anomaly rules

- An aborted run is EVIDENCE: fill the row with `ABORT` in commands_executed
  and the err line in aborts; do not delete or replace in place. If you need
  a third completing run, run a 4th and 5th — every row stays.
- Robot did not move at all: check the power switch / cells first (guide's
  first hypothesis), then note it — that is an abort row too.
- A run where fewer than 5 commands executed: keep the row, note the count —
  queue overflow is an honest drop and part of the findings.
