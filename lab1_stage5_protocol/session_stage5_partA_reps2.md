# Stage 5 session 1 — part A reps continued: L45 ×5, R45 ×3 (verbatim + analysis)

## Raw serial lines (verbatim)

L45 (second batch):
```
tturn cmd=L45.0 got=36.8deg L=-10cnt R=9cnt coast=-1cnt
tturn cmd=L45.0 got=13.6deg L=-6cnt R=1cnt coast=-5cnt
tturn cmd=L45.0 got=33.0deg L=-5cnt R=12cnt coast=3cnt
tturn cmd=L45.0 got=33.0deg L=-10cnt R=7cnt coast=-2cnt
tturn cmd=L45.0 got=44.6deg L=-10cnt R=13cnt coast=1cnt
```
R45:
```
tturn cmd=R45.0 got=-36.8deg L=13cnt R=-6cnt coast=2cnt
tturn cmd=R45.0 got=-40.7deg L=11cnt R=-10cnt coast=-1cnt
tturn cmd=R45.0 got=-40.7deg L=12cnt R=-9cnt coast=1cnt
```

## Cumulative 45° picture (10× L45, 3× R45)

| cell | n | mean got | mean E | σ |
|---|---|---|---|---|
| L45 | 10 | 34.1 | **−10.9** | **10.3** |
| R45 | 3 | −39.4 | **+5.6** | 2.3 |

F5 arithmetic holds on every new row (script-verified).

## The decisive pattern: the collapse is NOT fixed to one wheel

Per-wheel magnitudes across all 10 L45 turns (target ≈ 11.4 cnt/wheel):
- left: 8,12,12,12,1,10,6,5,10,10 → mean 8.6, collapsed (<8) in 3/10
- right: 8,4,12,12,12,9,1,12,7,13 → mean 9.0, collapsed (<8) in 3/10

**Both wheels collapse, in turns of BOTH directions, different wheels each
time.** R45 (3 reps) shows the same on the right (6,−10,−9 vs 11–13 left).
The plug-swap test is therefore likely to show "stays with the sketch, not
the motor" — but still worth running to exclude a marginal gearbox on BOTH
sides (unlikely).

## What the data now says

1. **45° checkpoint: FAILS as measured** (L45 σ 10.3, worst 13.6 = 70 % low;
   R45 better but mean −39.4 still −5.6 off with σ 2.3).
2. **Failures are per-START, not per-wheel:** each turn, one wheel
   (random which) fails to track from the start — the turn then "completes"
   when the good wheel alone reaches the stop differential, with the stalled
   wheel contributing only its kick/coast folds. Rows with diff ≥ 22.9 at
   the end (L45 row 5: 23; R90 earlier 42+) are the healthy ones.
3. **coast= correlates with the collapse side** (row 2: R=1, coast=−5;
   row 5 earlier: L=−1, coast=+5): the T_KICK reverse pulse slams the
   already-stalled wheel backwards past its breakaway — the fold records
   the wheel's REACTION, not the body's motion.
4. **Root-cause candidates, ranked:**
   a. **PWM_MIN=85 below the real per-wheel breakaway** for one wheel at a
      time (pack state unknown — volt never logged; row-9 precedent). The
      earlier floor-measurement (dead band ~75 + margin) was a different
      pack day.
   b. **Start asymmetry:** T_PULSE drives both wheels at PULSE_PWM=120;
      if one wheel breaks away and the other doesn't, the symmetry
      controller sees a magnitude error and DROPS the fast wheel's pwm
      toward the floor — helping the slow wheel stall longer (the
      equalisation fights breakaway). Rows 3/4 of the first L45 batch
      (−12/+12 clean) are the runs where both broke away together.
   c. Tyre/ground: dust or floor spot under ONE tyre at start (random per
      placement — matches "different wheel each time").

### Bench tests that discriminate (in order, ~10 min)
1. **`t` telemetry ON, run L45, paste tel lines**: tel prints pwmL/pwmR —
   shows whether the stuck wheel's pwm sits at 85 (floor clamp = controller
   asking more than the floor? no: clamp means pwm WANTED lower/higher) and
   whether its prog freezes while the other moves. ONE run discriminates
   controller-behaviour vs hardware-stick.
2. **K6 (raise K to 6) and repeat L45 ×3**: stronger equalisation makes (b)
   WORSE if (b) is real (fast wheel pulled down harder). Compare collapse
   rate 3/10 vs new.
3. **Charge/swap the pack, L45 ×3 again**: collapse rate should drop if (a).
   Volt log even without a meter: sketch has no volt read (A0-A3 used) —
   note pack age/charge state manually.
4. **Clean the tyres, different floor spot, L45 ×3**: discriminates (c).

### Do NOT
- retune trackMm (per-stop artifact, not ∝θ);
- raise PWM_MIN before test 1-3 (may mask the real mechanism and deadlock
  nothing but wastes the floor calibration);
- remove the kick (it fixed the pre-stage-4 run-on; its fold is honest).

### Stage-5 checkpoint status (protocol items unaffected)
Part A §3-§6 (skip rule, signs, range, queue/abort) remain runnable and
independent of this turn-accuracy issue — protocol items can still close
today; the 45° accuracy item belongs to stage-4's gate and waits for the
wheel fix.
