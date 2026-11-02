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

### 3.1 Five-command chain — queue dispatch proven (session 23.10)

Whole string `G0.8 L90 G1.2 R45 G0.6` sent as ONE line, no intervention.
Console verbatim:

    go cmd=G0.800m dir=fwd
    done cmd=G0.800m got=791.8mm err=-8.2mm L=156cnt R=154cnt
    go cmd=L90.0deg mode=spin
    tturn cmd=L90.0 got=89.2deg L=-20cnt R=26cnt coast=1cnt
    go cmd=G1.200m dir=fwd
    done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
    go cmd=R45.0deg mode=spin
    tturn cmd=R45.0 got=-27.1deg L=9cnt R=-5cnt coast=-4cnt
    go cmd=G0.600m dir=fwd
    done cmd=G0.600m got=597.7mm err=-2.3mm L=118cnt R=116cnt

| # | cmd | got | err | verdict |
|---|---|---|---|---|
| 1 | G0.800m | 791.8 mm | −8.2 mm | ✓ in tol; identical ticks to the 21.10 run (deterministic) |
| 2 | L90 spin | 89.2° | −0.8° | ✓ healthy: diff 46/46.4 cnt on the 1.94° grid; coast +1 folded |
| 3 | G1.200m | 1190.3 mm | −9.7 mm | ✓ byte-identical to the 4× repeatability burst (L=234 again) |
| 4 | R45 spin | −27.1° | −17.9° | ✗ COLLAPSE (limitation f): diff 14 of 23.2 cnt (60 %), wheels short of the 11.6/wheel nominal (L 9, R 5), coast −4 |
| 5 | G0.600m | 597.7 mm | −2.3 mm | ✓ |

- Strict go/done → go/tturn interleaving, zero err lines: the stage-5 queue
  dispatched a 5-command program end-to-end (checkpoint-5 "single string"
  exercised at 5-command depth; run-card §1.3's q/garbage literal still pending).
- Row 4 is limitation (f) LIVE on the frozen firmware: per-start collapse,
  E −17.9° — an order of magnitude past the 2.25° 5%-band. Run-card decision
  gate: a collapse run is the breakaway-patch trigger.
- End offset computed from odometry (no tape this session): after row 4 the
  heading is 62.1° instead of 45°, so row 5 ran in a rotated frame;
  reconstructed end ≈ (1088, 1719) mm vs predicted (1224, 1624) mm → ≈ 17 cm
  miss, dominated by the single collapse turn. [computed, not tape-measured]
- NOT the formal phase-2 acceptance: the 3× `G0.8 L90 G1.2` table above +
  B-zone tape + §4 spread remain open.

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
