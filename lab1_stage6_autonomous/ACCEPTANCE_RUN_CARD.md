# Stage 5+6 — ACCEPTANCE RUN CARD (freeze session, 21.10)

Firmware: flash `lab1_stage6_autonomous/lab1_stage6_autonomous.ino` (greeting
`stage6 v1: A-B autonomous | ...`). Constants FROZEN: k=5.108, b_spin=151,
TRACK_PIVOT=145 (prov), K_SYM=3.0, PWM_MIN=85. No calibration this session —
measure, log, decide, freeze.

PASS bands (pre-computed):
- turn 90° band: got within ±4.7° of 90 → collapse run if outside
- A→B stop: body inside the B zone (B-cross paper ±15 cm box) on 3/3 runs
- L−45/R−45: |got| within 45±5°, direction from the letter
- pivot 360: parked wheel |cnt| ≤ 2

## Step 1 — commands in order (~15 min)

1.1 WARM-UP (throwaway): `L90` — first turn of a session is unreliable; record
    but do not judge.
1.2 NEGATIVE RULE: `L-45` then `R-45`. Paste both tturn lines. PASS: turns
    LEFT / RIGHT respectively, |got| 40…50.
1.3 QUEUE+GARBAGE, one line: `qG1.2zL90`
    PASS order: `go cmd=G1.200m` → `done ...G1.200m` → `go cmd=L90.0deg` →
    `tturn ...L90.0`, q/z silently ignored (no err lines).
1.4 ABORT: send `G0.5 L90 G0.5` (one line), then WHILE leg 1 runs send `x`.
    Record. If motion continues, send `b`. Report: which key stopped a queued
    run, and whether the rest of the queue (L90, G0.5) executed or died.
    (Note: `x` is bench-mode toggle in this firmware — expected verdict: `b`
    is the emergency stop and it flushes the queue; fill the table from what
    you see.)
1.5 PIVOT GATE (optional): `m` → `L360` → `R360` → paste both tturn + the
    parked-wheel counts; with a tape pointer on the robot measure heading
    offset δ_p after 360° and record it (track_pivot_true = 145×360/(360+δ_p)).
    Then `m` back to spin.
1.6 A→B ACCEPTANCE ×3: mark A (start + heading tape) and B (paper cross at the
    predicted end of `G0.8 L90 G1.2`: leg1 800 mm, 90° left, leg2 1200 mm —
    end ≈ (800, 1200) mm in the start frame). ONE line per run:
    `G0.8 L90 G1.2`
    Between runs: carry back to A, align on the tape, touch nothing mid-run.
    Record per run: full console chain (strict go/done → go/tturn → go/done)
    and stop position vs B in cm: left/right × short/long.

## Step 2 — decision gate (I apply this from your paste)
- 3/3 in B zone AND no turn off ≥4.7° → FREEZE (patch stays unused)
- any turn off ≥4.7° (collapse) → breakaway patch, re-run the 3 lines, freeze
  regardless of outcome

## Paste back
The raw console output of 1.1–1.6 verbatim + the 3 B-offsets + δ_p if measured.
