# Odometry on the AlphaBot robot — Arduino

ISS Lab 1: odometry on a Waveshare AlphaBot (Arduino UNO, ATmega328P) with
slotted wheel encoders. All firmware is written from scratch — no
manufacturer libraries (no `AlphaBot.h`, no E01–E12 examples), per the
assignment's ground rules. Only `Arduino.h`, `avr/interrupt.h`,
`util/atomic.h`, `stdlib.h`, `string.h`.

## What it does

The acceptance criterion: the operator drives the robot from point A to
point B once; the robot is carried back to A; one command string sent over
serial then drives it to B autonomously. Final acceptance: **3/3 runs of
`G0.8 L90 G1.2 R45 G0.6` completed, all commands within tolerance, zero
aborts** — see `stage6_acceptance_report.md`.

## Stages

| stage | folder | what it covers |
|---|---|---|
| 1 | `lab1_stage1_motor_test/` | motor layer (H-bridge, signed PWM, coast vs brake), dead band, run-on |
| 2 | `lab1_stage2_encoders/` | wheel encoders, k calibration (5.1087 mm/count, spread 1.54 %), PWM→speed characteristic |
| 3 | `lab1_stage3_straight/` | straight-line controller: position-driven ramps + proportional symmetry controller, wheel guard |
| 4 | `lab1_stage4_turns/` | turn controller (spin/pivot), track calibration (b_spin 151 mm), angular resolution analysis |
| 5 | `lab1_stage5_protocol/` | serial protocol `G<metres> L<deg> R<deg>` — non-blocking parser, program queue, result lines |
| 6 | `lab1_stage6_autonomous/` | autonomous A→B execution (frozen firmware, 8-command queue) |
| 7 | `lab1_stage7_faultrecovery/` | wheel-sync fault layer: one-wheel stall → brake+test-pulse+resync, slip detection (MIN-progress), both-wheels-blocked abort |

Each stage folder has the `.ino` plus `REPORT_STAGE*.md` / `MEASURE_*.md`
with the measurement data, tuning records and checkpoint status.

## Key calibration values

| constant | value | provenance |
|---|---|---|
| K_MM (mm per encoder count) | 5.1087 | floor calibration, 4 runs, spread 1.54 % |
| SLOTS / N_REV | 20 / 40 (both edges) | counted + electrically verified |
| b_spin (effective track) | 151.0 mm | 1440° multi-rev calibration |
| b_pivot | 145.0 mm | provisional — see open items |
| PWM_MIN / PWM_CRUISE | 85 / 130 (drive), 110 (turn) | floor dead-band + stage-2 characteristic |
| K_SYM (symmetry gain) | 3.0 PWM/mm | floor-tuned |

## Reports & analysis

- `stage6_acceptance_report.md` — the graded acceptance: 3 runs, statistics,
  real-world fault-recovery event
- `NONDET_ANALYSIS.md` — L90 spin non-determinism root-caused to a left-wheel
  breakaway failure (not a firmware state leak), with proposed fixes
- `L45_ANOMALY_ANALYSIS.md`, `PIVOT_STRAIGHT_ANALYSIS.md` — short-turn
  collapse and pivot under-rotation analyses
- `CALIBRATION_AUDIT.md` — cross-file consistency audit of every constant
- `stage1_runon_matrix_template.md`, `stage3_fillins_template.md`,
  `stage4_accuracy_template.md`, `stage6_acceptance_template.md` —
  measurement templates (partially filled with real session data,
  `<<unmeasured>>` marks everywhere data is still owed)
- `NEXT_ROBOT_SESSION.md` — executable checklist for the next floor session

## Tools

`tools/` — stdlib-only Python post-processing:
`stage2_process.py`, `stage3_tables.py`, `stage4_process.py` (turn-accuracy
statistics + 45° gate), `stage6_process.py` (per-command acceptance
statistics, phase-3 error separation).

`_hostcheck/` — host-side simulation harness (no hardware needed): the
controller logic compiled for the desktop with a simulated robot plant,
regression suites and verification logs.

## Open items

- tape measurements of final position vs B (visual confirmation recorded
  for the acceptance; scale_error pending if required)
- left-wheel breakaway retry in the turn controller (proposal in
  `NONDET_ANALYSIS.md`)
- TRACK_PIVOT 360° multi-rev calibration
