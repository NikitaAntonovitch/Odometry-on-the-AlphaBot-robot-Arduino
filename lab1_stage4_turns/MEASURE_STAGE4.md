# Stage 4 — measurements to take at the lab (fill this in on the robot)

Sketch to flash: `lab1_stage4_turns/lab1_stage4_turns.ino`. Serial: **9600**.
Commands: `L<deg>` / `R<deg>` turn left/right (T/B also accepted), `m` cycles
turn mode spin → pivotL → pivotR → spin, `J<mm>` sets the effective track
(for calibration), `K<gain>` symmetry gain, `t` telemetry, `e` encoders,
`c`/`b` stop, `h` menu.

Sign convention (adopted, recorded in the report): **positive angle = left =
counter-clockwise seen from above**. The command letter carries the direction;
a negative number is taken as its absolute value (same rule as F/B in stage 3).

Robot prep: charged cells, protractor or a marked circle on the floor (a sheet
of paper with radii drawn every 10° works), tape for the track estimate.
Battery voltage not measured (no multimeter — standing note).

## 1. Track calibration by multi-revolution (guide p.24 — do this FIRST)

`m` until `turn mode=spin`, then `L1440` (= 4 full revolutions; 5 recommended
by the guide, 4 keeps the robot inside a desk-scale area). When it stops, measure
the actual final heading vs the start mark (protractor or the marked circle).
Repeat `R1440`. The deviation spreads over 1440°, so 1° of reading error is
negligible. Correction: `b_calibrated = b_current × 1440 / actually_achieved`.

| dir | commanded | achieved (protractor) | correction factor | b_used (mm) |
|---|---|---|---|---|
| L1440 | 1440° | ______° | ______ | 160.0 × f |
| R1440 | 1440° | ______° | ______ | |

- after computing: `J<new track>` and re-run L1440 — achieved should now be ~1440
- difference L vs R correction sign → caster friction asymmetry (note it)
- tape measurement of the track: ______ mm; difference vs calibrated: ______
  (expected — contact point shift, slip, caster; explanation in report sec.3)

## 2. Angular resolution work-out (no robot needed — from the numbers)

With k = 5.1087 mm/count and the calibrated track b:
- spin: 1 count = 2k/b body-degrees = ______° (sim said ~0.46° at b=160)
- pivot: 1 count = k/b body-degrees = ______° (pivot resolves TWICE as finely)
- counts per wheel for 15° spin: ______ (sim: ~8) — vs STOP_TOL 0.7°: the
  tolerance is ~1.5 counts; the guide's "15° lies close to the resolution limit"
- counts per wheel for 30/45/90° spin: ______ / ______ / ______

## 3. Turn accuracy — spin mode (guide table: 15/30/45/90 × L/R × 5 reps)

For each cell: note the `got=` value from the `tturn` line (odometry truth) AND
the protractor reading. The table below takes 5 reps per cell; `got` and
protractor in separate columns so controller error and real-world error separate.

| angle | dir | rep | got (deg) | protractor (deg) |
|---|---|---|---|---|
| 15 | L | 1..5 | | |
| 15 | R | 1..5 | | |
| 30 | L | 1..5 | | |
| 30 | R | 1..5 | | |
| 45 | L | 1..5 | | |
| 45 | R | 1..5 | | |
| 90 | L | 1..5 | | |
| 90 | R | 1..5 | | |

(multiple rows per cell — leave space; mean and spread computed at the end)

## 4. Turn accuracy — pivot mode, at least the small angles

`m` → `turn mode=pivotL`. Same table for 15/30 (L only is fine, or both):

| angle | mode | rep | got | protractor |
|---|---|---|---|---|
| 15 | pivotL | 1..5 | | |
| 30 | pivotL | 1..5 | | |

Comparison of the two modes at 15° is THE report argument: pivot resolves
angles 2× better per count — if pivot's spread at 15° is visibly smaller,
the spin limitation is resolution, not control (report sec.5).

## 5. Diagnosis table (guide p.24) — fill after sec. 3

| observation | your verdict |
|---|---|
| mean error grows ∝ angle (track under-calibrated)? | |
| mean error constant (run-on — ramp/speed fix)? | |
| spread unchanged under any tuning (resolution limit)? | |

## 6. Enter into the sketch when done

```
static const float TRACK_MM = ___.0f;  // calibrated (sec. 1), not tape
```
(J<mm> is runtime — for the permanent value update TRACK_MM and reflash)
