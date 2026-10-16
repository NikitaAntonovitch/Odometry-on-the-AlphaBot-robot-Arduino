# Stage 3 — measurements to take at the lab (fill this in on the robot)

Sketch to flash: `lab1_stage3_straight/lab1_stage3_straight.ino`.
Serial monitor: **9600**. Commands: `F<mm>` forward, `B<mm>` backward,
`K<gain>` symmetry gain, `t` telemetry, `e` encoders, `z` zero, `c` coast,
`b` brake, `h` menu. Battery voltage: not measured (no multimeter — same note
as stage 2; keep the session short so the supply does not drift mid-session).

Before starting: floor marks (start line + lateral reference line), a metre
ruler, something to mark the stopping position of the wheel axis.

## 1. First sanity on the robot (5 min)

| test | how | expected | result |
|---|---|---|---|
| link sanity | open monitor, press `h` | menu appears, one line | ☐ |
| F100 | `F100`, watch, then `e` | drives ~100 mm, done line, no jerk at start | ☐ |
| B100 | `B100` | same backward | ☐ |
| done line format | read the `done cmd=... got=... err=... L= R=` line | copyable, plausible numbers | ☐ |

## 2. Ramp behaviour (checkpoint 3: "neither jerks at the start nor overshoots")

| observation | how it should look | result |
|---|---|---|
| start | wheels spin up over ~60 mm, no visible lurch / wheel slip chirp | ☐ |
| stop | robot eases down and stops ON the mark, no coast-past | ☐ |
| overshoot at F200/F500 | wheel axis stops within ~1 cm past the mark, not more | ☐ |

If the robot overshoots: increase `DEC_MM` (60) in the sketch and re-flash.
If it still creeps past: raise `STOP_TOL_MM` slightly (10 → 12).

## 3. Gain tuning (checkpoint 3: at least 4 gains with symptoms)

For each gain: `K<val>`, then `F500`, watch the line, then `e` and note.
Repeat with `K0` FIRST (baseline, no correction). Reset the robot on the start
line, same orientation every run.

| gain | path shape (watch from above) | heading feel | lateral offset at 500 mm (mm) | notes |
|---|---|---|---|---|
| 0   | curves right (R wheel faster ~7 %) | | | baseline |
| 0.3 | | | | |
| 0.6 | | | | default |
| 1.5 | | | | |
| 3   | | | | may oscillate / zigzag near start (small headroom above dead band during ramp) |

- too-low gain: visible right-hand curve, offset grows with distance
- good gain: straight within a cm over 0.5 m
- too-high gain: wheels audibly fight each other, zigzag, or a wheel stalls
  during the ramp (correction clamped by CORR_MAX on purpose)

## 4. Lateral deviation, zero gain vs chosen gain (checkpoint 3)

Run `F500` five times per configuration, robot starts on the same mark each
time. Measure how far the wheel axis ends from the start line's parallel,
perpendicular to the path.

| run | K=0 offset (mm, + = right) | chosen gain offset (mm) |
|---|---|---|
| 1 | | |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |
| mean | | |

## 5. Symmetry error vs time for two gains (checkpoint 3)

`K0.3` then `t` then `F500`; capture the `tel` lines (copy from the monitor).
Repeat with the chosen gain. Save both logs — they go into the report as the
error-vs-time evidence. (Telemetry runs ~45 chars per 100 ms — fine at 9600.)

## 6. Controller error vs scale error (guide p.21)

At the chosen gain, for each distance: note `got` (odometry) AND the true
position measured on the floor.

| commanded | got (odometry, mm) | odometric error = got−cmd | true (floor, mm) | scale error = true−got |
|---|---|---|---|---|
| F200 | | | | |
| F500 | | | | |
| B500 | | | | |

- odometric error big and one-signed → controller/tolerance issue
- true differs from got consistently in % → k (scale) correction:
  k_new = k_old × (mean true / mean got). If confirmed, update `K_MM` and say
  so in the report (guide: "controller error and scale error separated; k corrected")

## 7. Enter into the sketch when done

```
static const float K_SYM       = ?.??f;  // chosen symmetry gain (section 3)
static const float DEC_MM      = ??.0f;  // if the stop overshoot needed tuning (section 2)
static const float STOP_TOL_MM = ??.0f;  // if tolerance needed tuning (section 2)
static const float K_MM        = ...;    // ONLY if section 6 justified a scale correction
```
