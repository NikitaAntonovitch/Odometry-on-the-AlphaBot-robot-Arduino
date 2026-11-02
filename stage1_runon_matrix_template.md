# Stage 1 — run-on matrix template (the missing 2×2, coast vs brake)

Status: TEMPLATE ONLY — every value is `<<unmeasured>>` until the robot
session writes it. This is the single open Stage-1 checkpoint item (◑ in
REPORT_STAGE1 §5): run-on distance measured for coasting AND dynamic braking,
at TWO PWM values (medium and high), with a tape.

Firmware: `lab1_stage1_motor_test.ino` (the stage-1 bench sketch — flash it
back for this block; keys below). Do this block on the FLOOR (guide: the
stand threshold does not transfer).

## L298 background (why the two stop modes differ)

The H-bridge channel has four output states (guide p.5):

- **coasting** — outputs disconnected, winding open: the spinning motor is a
  generator with no load path; kinetic energy dissipates in friction only →
  LONG run-on.
- **dynamic braking** — winding shorted through one pair of switches (both
  direction inputs of the channel at the same level, enable high): the
  back-EMF drives current through the winding resistance, dissipating the
  energy electrically → SHORT run-on.

On the bench sketch: `c` = coast stop, `b` = brake. The measurement answers
the guide's question of why "target reached → stop" is not enough in stage 3.

## Procedure (per cell)

1. Mark a start line; mark the kill line at the target distance (tape).
2. Send the drive command and let the robot cross the kill line at steady
   speed (drive mode keys `1`/`2` at the PWM set with `+`/`-`, or the preset
   mode keys — see the sketch menu `h`).
3. AT the kill line press `c` (coast) or `b` (brake) — one press, no repeat.
4. Measure with the tape from the KILL LINE to where the wheel axis stopped:
   that is the run-on distance. Time it with a stopwatch (phone): press at
   the line, stop at rest — run-on time in ms (±100 ms honest).
5. One row per cell; repeat each cell twice if time permits and keep both
   numbers (mean + spread).

Medium PWM = 90, high PWM = 170 (the stage-2 characteristic's mid and upper
points; both are inside the linear region).

## The 2×2 matrix

| motion | coast_med_PWM 90 | coast_high_PWM 170 | brake_med_PWM 90 | brake_high_PWM 170 |
|--------|------------------|--------------------|--------------------|--------------------|
| run-on distance (mm) | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| run-on time (ms) | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

Second reps (optional, same cells):

| motion | coast_med_PWM 90 | coast_high_PWM 170 | brake_med_PWM 90 | brake_high_PWM 170 |
|--------|------------------|--------------------|--------------------|--------------------|
| run-on distance (mm) rep 2 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| run-on time (ms) rep 2 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

Pack voltage at start: ______ V (same-state-of-charge rule — a sagging pack
shifts speed at the same PWM and invalidates comparisons mid-table).

## What the numbers decide (fill after the session)

- Expected ordering: coast ≫ brake at both PWMs; run-on grows with PWM.
  If brake ≈ coast: the brake path is not actually shorting the windings —
  check the brake() pin states against the L298 datasheet before blaming
  mechanics.
- Stage-3 consequence: DEC_MM (60) was sized on "run-on almost removed at
  the V_FLOOR arrival speed" — the med-PWM brake cell is the number that
  tests that claim. If brake_med run-on > ~15 mm, the stage-3 DEC_MM/STOP_TOL
  re-tune flag (stage3_fillins §a) is triggered.
- This closes checkpoint-1 item 6 completely → REPORT_STAGE1 §5 ◑ → ☑.
