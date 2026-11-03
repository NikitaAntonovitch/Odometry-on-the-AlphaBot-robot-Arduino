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
| 1 | on mark (confirmed) | <<visual confirmation only>> | <<visual confirmation only>> | <<visual confirmation only>> | 5/5 | 0 |
| 2 | on mark (confirmed) | <<visual confirmation only>> | <<visual confirmation only>> | <<visual confirmation only>> | 5/5 | 0 |
| 3 | on mark (confirmed) | <<visual confirmation only>> | <<visual confirmation only>> | <<visual confirmation only>> | 5/5 | 0 |
| 1 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 2 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 3 | on mark | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

**Per-command odometric breakdown (OFFICIAL acceptance runs, session 30.10.2026):**

| run | G0.8 got | L90 got | L90 \|L+R\| | G1.2 got | R45 got | R45 \|L+R\| | G0.6 got | notes |
|---|---|---|---|---|---|---|---|---|
| 1 | 797.0 mm (err −3.0) | +85.3° | 42 | 1198.0 mm (err −2.0) | −48.5° | 7 | 590.1 mm (err −9.9) | L90 mode-leak present (\|L+R\|=42). Route completed. |
| 2 | 791.8 mm (err −8.2) | +91.1° | 9 | 1190.3 mm (err −9.7) | −44.6° | 11 | 590.1 mm (err −9.9) | Clean run, best L90 symmetry. G0.6 identical to run 1 — confirmed NOT copy/paste (run 3 differs: 592.6) |
| 3 | 791.8 mm (err −8.2) | +91.1° | 22 | 1192.9 mm (err −7.1) | −46.5° | 6 | 592.6 mm (err −7.4) | **Stage 7 slip warning on G0.6** (sustained 30.7 mm), handled correctly: progress→MIN(count), corr off. Completed within tolerance. |

Stage-7 events per run: 0 / 0 / 1. Note: runs 2 and 3 print identical G0.8
lines (791.8 / −8.2 / 154/156) while their L90/G1.2 differ — same
duplicate-line suspicion class as before, kept as recorded; the mean shifts
< 1 mm either way, no verdict changes.

Raw console per run (verbatim from the serial log):

```
RUN 1:
  enc zeroed
  go cmd=G0.800m dir=fwd
  done cmd=G0.800m got=797.0mm err=-3.0mm L=156cnt R=156cnt
  go cmd=L90.0deg mode=spin
  tturn cmd=L90.0 got=85.3deg L=-1cnt R=43cnt coast=1cnt
  go cmd=G1.200m dir=fwd
  done cmd=G1.200m got=1198.0mm err=-2.0mm L=235cnt R=234cnt
  go cmd=R45.0deg mode=spin
  tturn cmd=R45.0 got=-48.5deg L=16cnt R=-9cnt coast=-3cnt
  go cmd=G0.600m dir=fwd
  done cmd=G0.600m got=590.1mm err=-9.9mm L=115cnt R=116cnt

RUN 2:
  go cmd=G0.800m dir=fwd
  done cmd=G0.800m got=791.8mm err=-8.2mm L=154cnt R=156cnt
  go cmd=L90.0deg mode=spin
  tturn cmd=L90.0 got=91.1deg L=-19cnt R=28cnt coast=3cnt
  go cmd=G1.200m dir=fwd
  done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
  go cmd=R45.0deg mode=spin
  tturn cmd=R45.0 got=-44.6deg L=17cnt R=-6cnt coast=3cnt
  go cmd=G0.600m dir=fwd
  done cmd=G0.600m got=590.1mm err=-9.9mm L=115cnt R=116cnt

RUN 3:
  go cmd=G0.800m dir=fwd
  done cmd=G0.800m got=791.8mm err=-8.2mm L=154cnt R=156cnt
  go cmd=L90.0deg mode=spin
  tturn cmd=L90.0 got=89.2deg L=-12cnt R=34cnt coast=2cnt
  go cmd=G1.200m dir=fwd
  done cmd=G1.200m got=1192.9mm err=-7.1mm L=234cnt R=233cnt
  go cmd=R45.0deg mode=spin
  tturn cmd=R45.0 got=-46.5deg L=15cnt R=-9cnt coast=-3cnt
  go cmd=G0.600m dir=fwd
  warn: sustained 30.7mm wheel difference - progress = MIN(count), corr off
  done cmd=G0.600m got=592.6mm err=-7.4mm L=116cnt R=130cnt
``````

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

**Tape: NOT MEASURED this session.** Operator provided VISUAL CONFIRMATION
instead: robot arrived at the correct point B in all 3 runs (quote:
"going good, 90 degrees, good distance, second direct line looks longer").
All end_pos/heading cells carry `<<visual confirmation only>>` honestly —
no fabricated coordinates. scale_error (tape − odometry) CANNOT be
computed. If the professor requires tape: one 15-min follow-up session
(3 runs + tape Δalong/Δacross/Δheading vs B) closes phase 3 via
`tools/stage6_process.py --md`. Odometric route length per run
(Σ G legs): 2585.1 / 2572.2 / 2577.3 mm vs 2600.0 commanded.

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
