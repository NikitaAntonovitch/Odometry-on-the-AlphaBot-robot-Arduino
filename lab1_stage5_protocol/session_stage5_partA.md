# Stage 5 session 1 — part A raw log (user paste, verbatim) + desk analysis

## Raw serial lines (verbatim)

G0.2:
```
go cmd=G0.200m dir=fwd
done cmd=G0.200m got=196.7mm err=-3.3mm L=39cnt R=38cnt
```
G-0.2:
```
go cmd=G-0.200m dir=back
done cmd=G-0.200m got=-199.2mm err=0.8mm L=-40cnt R=-38cnt
```
L90:
```
go cmd=L90.0deg mode=spin
tturn cmd=L90.0 got=91.1deg L=-24cnt R=23cnt coast=-1cnt
```
R90:
```
go cmd=R90.0deg mode=spin
tturn cmd=R90.0 got=-81.4deg L=22cnt R=-20cnt coast=2cnt
```
L45:
```
go cmd=L45.0deg mode=spin
tturn cmd=L45.0 got=31.0deg L=-8cnt R=8cnt coast=-2cnt
```

## Desk analysis (desk-side, not robot-verified beyond these lines)

### Drives — PASS
- G0.2: got 196.7, err −3.3. Above the [-10,-6] window like stage-3 row 7
  (err −3.7): stop fired in the same tick past the edge — consistent, not a bug.
  Wheel symmetry R/L = 38/39 = 0.974 (excellent).
- G−0.2: got −199.2, err +0.8. Tick-edge landing on the good side. R/L 0.95.
- F5 determinism + symmetry controller confirmed on hardware for straights.

### Turns — got= internally consistent with counts (F5 holds), but TWO anomalies
Arithmetic check (all pass, verified by script): got = (cR−cL)·k/b·180/π
with k=5.1087, b=151:
- L90: (23−(−24)) = 47 diff cnt → 91.1° ✓ E = +1.1 (inside gate +3.5…+5.5? —
  BELOW the gate floor but the right order; gate was set for run-on-dominant E)
- R90: (−20−22) = −42 diff cnt → −81.4° ✓ E = −8.6  ← UNDERSHOOT 8.6°
- L45: (8−(−8)) = 16 diff cnt → 31.0° ✓ E = −14.0  ← FAILS the 45° requirement
  IF the physical angle matches odometry

Reference: stop fires at prog ≥ tgt−0.7 BY CONSTRUCTION (90° → ~23.0 cnt/wheel,
45° → ~11.4 cnt/wheel). Final counts BELOW that (L45: 8/8, R90: 22/21) prove
counts were REMOVED after the stop trigger — i.e. the mandatory active-brake
kick (KICK_PWM 95, 60 ms) back-drove the wheels and F5 folded that real reverse
rotation into got=:
- L45: missing 14.0° = 3.6 differential counts of reverse fold
- R90: missing 8.6° = 2.2 differential counts
- L90: coast=-1 only, minimal fold, E stayed +1.1
Asymmetry L vs R of the bite = caster/stiction direction interaction (one of
the guide's three track-asymmetry reasons). L90 vs R90 9.7° apart would FAIL
the stage-4 §G gate-2 clause "L/R means within 2°" if it reproduces.

### NOT a calibration issue
Track error scales WITH angle (E∝θ); this bite is nearly CONSTANT in counts
(−3.6 vs −2.2 while counts at stop differ 2×) — a per-stop kick artifact,
not a scale error. Do NOT touch trackMm from this data.

### Missing evidence (needed from the bench)
1. PHYSICAL angle after L45 (protractor, robot untouched): ~31° = body really
   unwound (execution issue → kick too strong) vs ~45° = wheels skidded during
   the kick (encoder counted, body didn't follow → got= misreports, fold rule
   needs a look). THIS SINGLE NUMBER DECIDES THE FIX PATH.
2. Reproducibility: L45 ×3 and R90 ×3 with protractor readings.
3. Protractor column for every turn (worksheet §1 asks for it).

### Stage-5 checkpoint status from part A alone
- one result line per command, turn lines carry got=: PROVEN (5/5 commands)
- G signed metres: exercised (G−0.2 ran); L/R abs rule: pending (L−45/R−45)
- turn ACCURACY is stage-4 gate territory: this data feeds it, does not close it
