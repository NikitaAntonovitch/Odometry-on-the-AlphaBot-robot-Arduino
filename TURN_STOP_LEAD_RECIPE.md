# TURN_STOP_LEAD_DEG — recipe verification (task item 8, desk pass 02.11.2026)

## Location

`TURN_STOP_LEAD_DEG` exists in exactly ONE firmware: the stage-7 sketch.

- lab1_stage7_faultrecovery/lab1_stage7_faultrecovery.ino:73
- Used at :846 — `if (rem <= STOP_TOL_DEG + TURN_STOP_LEAD_DEG) { stopTurn(); }`
  (stop BEFORE the target so the brake residual + stop-step overshoot land
  ON the target).
- Stages 4/5/6 do NOT have this constant: their turn stop is the
  PULSE→RUN→KICK→COAST phase machine where the active reverse kick + coast
  poll absorb and COUNT the run-on, so the stop allowance is just
  STOP_TOL_DEG (0.7°) — see lab1_stage6_autonomous.ino:520-522 and the
  stage-4 design note. Consequently this recipe is only exercised on the
  stage-7 firmware (Block C of NEXT_ROBOT_SESSION.md).

## Current value and the comment block (verbatim, .ino:72-78)

```cpp
static const float STOP_TOL_DEG = 0.7f;   // ~1.5 counts at the calibrated track
static const float TURN_STOP_LEAD_DEG = 7.8f; // stop this much EARLY: brake residual + stop-step
                                              // overshoot at the PWM_MIN floor (measured on the
                                              // robot as got= minus target with lead=0; sim brake
                                              // plant: 2 count-pairs = 7.75 deg). recipe: flash
                                              // with 0.0, run L90, set lead = (got - target),
                                              // re-flash. quantisation floor: 1 pair = 3.9 deg
```

## Verdict on the in-source recipe

The comment DOES contain a recipe, but it is terse and ambiguous on four
points: (1) it does not say how many runs, (2) it does not exclude collapse
turns from the mean, (3) it does not say what to do with the quantisation
floor (3.9°/pair) when the measured lead is below one pair, (4) it does not
say how to validate the re-flashed value. Per the task brief the precise
recipe was therefore written OUTSIDE the firmware (no .ino edit in this
pass) as **Block C of NEXT_ROBOT_SESSION.md**. Its content, in short:

1. Flash the stage-7 sketch with the constant temporarily 0.0 (one-line
   re-flash; this IS the measurement, not a tuning change).
2. Run `L90` ×5 and `R90` ×5; copy every `tturn got=`.
3. Use HEALTHY runs only for the mean (both wheels broke away: per-wheel
   counts ≈ 23±2 at L90 — a collapsed turn measures the breakaway failure,
   not the brake plant). Annotate, do not delete, the discarded rows.
4. lead_new = |mean got| − 90. If |lead_new| < 3.9, clamp to 3.9 (one
   count-pair — the resolution floor already noted in the source comment).
5. Decision grid: |lead_new − 7.8| ≤ 1.9 (half a pair) → value 7.8 stands,
   re-flash the original. Otherwise the measured value is recorded in the
   report + CALIBRATION_AUDIT note and the .ino edit happens at the desk
   AFTER the session, followed by a confirm run (`L90` ×3, mean |E| ≤ one
   pair).

The value 7.8 itself was NOT changed here — it is robot-side (the brief and
the constant's own comment both say so).

## Consistency cross-checks done

- 7.8 ≈ 2 count-pairs × 3.88°/pair = 7.75° at b_spin 151 (comment's sim
  figure) — consistent with the 1.94°/count grid.
- STOP_TOL_DEG 0.7 + 7.8 = 8.5° stop window — matches the healthy L90 pool
  behaviour (mean E +0.1°, REPORT_STAGE5 §2) where got lands ≈ 82–93°.
- The constant is float and only compared against `rem` (deg) — no unit
  mismatch with STOP_TOL_DEG.
