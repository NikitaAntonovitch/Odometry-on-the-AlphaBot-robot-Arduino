# Stage 6 — FINAL REPORT (freeze edition, 21.10)

Firmware: `lab1_stage6_autonomous.ino` tag **stage6-v1-FROZEN** — logic
IDENTICAL to stage5-v1 (diff-proven: zero code delta outside the
greeting/menu/header), so all stage-5 numbers carry over. k = 5.108,
b_spin = 151, TRACK_PIVOT = 145 (prov), K_SYM = 3.0.

## 1. Architecture (per the guide's table)

| block | from | when it runs |
|---|---|---|
| odometry | stage 2 | interrupts (CHANGE, both edges), atomic SREG-restore reads |
| straight controller | stage 3 | 50 Hz fixed period (position-driven profile + symmetry) |
| turn controller | stage 4 | 50 Hz fixed period (phase machine: PULSE→RUN→KICK→COAST) |
| command reading | stage 5 | every loop() pass, non-blocking |
| execution | variant B | 8-command program queue, dispatchStep when idle |

Servicing order chosen: read → dispatch → control step. Nothing in the control
loop stalls execution (no delay-based motion anywhere).

## 2. RX-buffer math (checkpoint item, computed before phase 2)

- acceptance route `G0.8 L90 G1.2` = 13 chars (+1 newline) = **14 B**
- Arduino core UART rx buffer = **64 B** → the string fits with 50 B margin →
  sending the whole route at once is safe.
- execution variant B makes this non-critical anyway: the program queue holds
  3 commands (≤ 8 cap) regardless of link timing.

## 3. A→B acceptance (phase 2/3 — fill from the acceptance run card)

Route: `G0.8 L90 G1.2` (A → 0.8 m → 90° left → 1.2 m → B). Predicted end
(odometry frame): (800, 1200) mm from A.

| run | console chain (strict order?) | stop vs B (cm) | verdict |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |

- Decision gate: 3/3 in B zone AND no turn off ≥4.7° → FREEZE (patch unused);
  any collapse run → breakaway patch, re-run, freeze regardless. Verdict: ______

## 4. Phase-3 error separation (tools/stage6_process.py)

Fill after the 3 runs (`python tools/stage6_process.py --x1 .. --y1 .. ...`):
mean shift = systematic (calibration: k along-route, track across); spread σ =
random (resolution + slip — tuning will not help). Odometry-vs-tape route
length = calibration error, not controller error.

## 5. Why phase 2 differs from phase 1 (guide's table, our evidence)

- **no pause between commands** in phase 2 (phase 1 had operator thinking
  gaps) → different thermal/stiction state per leg; measured today as the
  per-start wheel collapse (p ≈ 0.30) and the cold-start first-turn effect.
- **accumulation of heading error:** every turn contributes up to ±3.9° of
  quantisation landing; over the route the B offset grows with the remaining
  leg length (1.2 m × sin 3.9° ≈ 8 cm worst-case from turn quantisation alone).
- **pack voltage drift** between phase 1 and phase 3 runs (voltage not logged —
  standing note); the dead band and ramps shift with it (row-9 precedent).
- **start mark reproduction** dominates: the guide's warning is quantified by
  our abort-free protocol — a 5° alignment error at A puts B off by ~10 cm.

## 6. Known limitations

Verbatim list shared with REPORT_STAGE5.md §3 (a–f): stop transient, cold-start
stall, R-backward weakness + sum-regulation, abort-test result (fill from run
card §1.4), b/TRACK_PIVOT odometry-only validation, per-start collapse with the
designed-but-unapplied patch (prediction on file).

## 7. Checkpoint 6 — final state

- [x] drive and turn commands each end with a result line (stage-5 session)
- [x] whole sequence from a single string (stage-5 §2 + A→B chains here)
- [x] unrecognised characters cause no motion (stage-5 §3)
- [x] out-of-range handled per the adopted rule (stage-5 §5)
- [x] units + sign rule documented (REPORT_STAGE5 §1, robot-verified)
- [x] phase 1 route established with single commands, sequence recorded
- [x] string length vs buffer capacity computed (§2 above)
- [ ] phase 2 autonomous drive ×3 — fill from run card §1.6
- [ ] phase 3 spread measured, components separated — fill via stage6_process.py
- [ ] phase-2-vs-phase-1 explanation — §5 above, quantified after the runs
