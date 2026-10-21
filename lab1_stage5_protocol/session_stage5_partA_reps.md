# Stage 5 session 1 — part A continuation: turn accuracy reps (verbatim + analysis)

## Raw serial lines (verbatim, 5 reps per cell)

L90:
```
tturn cmd=L90.0 got=71.7deg L=-19cnt R=18cnt coast=-3cnt
tturn cmd=L90.0 got=91.1deg L=-22cnt R=25cnt coast=1cnt
tturn cmd=L90.0 got=85.3deg L=-24cnt R=20cnt coast=-2cnt
tturn cmd=L90.0 got=77.5deg L=-22cnt R=18cnt coast=-3cnt
tturn cmd=L90.0 got=83.4deg L=-22cnt R=21cnt coast=-2cnt
```
R90:
```
tturn cmd=R90.0 got=-81.4deg L=20cnt R=-22cnt coast=-2cnt
tturn cmd=R90.0 got=-83.4deg L=20cnt R=-23cnt coast=-2cnt
tturn cmd=R90.0 got=-64.0deg L=17cnt R=-16cnt coast=4cnt
tturn cmd=R90.0 got=-77.5deg L=20cnt R=-20cnt coast=2cnt
tturn cmd=R90.0 got=-79.5deg L=20cnt R=-21cnt coast=-2cnt
```
L45:
```
tturn cmd=L45.0 got=31.0deg L=-8cnt R=8cnt coast=-2cnt
tturn cmd=L45.0 got=31.0deg L=-12cnt R=4cnt coast=-4cnt
tturn cmd=L45.0 got=46.5deg L=-12cnt R=12cnt
tturn cmd=L45.0 got=46.5deg L=-12cnt R=12cnt
tturn cmd=L45.0 got=25.2deg L=-1cnt R=12cnt coast=5cnt
```

## Desk analysis (verified by script; got = (cR−cL)·k/b holds on every row)

| cell | mean got | mean E | σ | range |
|---|---|---|---|---|
| L90 | 81.8 | **−8.2** | 7.5 | 71.7…91.1 |
| R90 | −77.2 | **+12.8** (|E| 12.8) | 7.7 | −64.0…−83.4 |
| L45 | 36.1 | **−9.0** | 9.8 | 25.2…46.5 |

Reference: stop fires at diff ≥ 46.1 cnt (90°) / 22.9 cnt (45°) BY CONSTRUCTION.

### Mechanism split — TWO separate problems, neither is track calibration

**(A) Stop-fold bite (~−2 diff counts, both wheels similar):** rows ending
1-2 counts under the stop trigger with coast=±2..4 (L90 rows 1/3/4, R90 rows
1/2/4/5). The mandatory T_KICK reverse pulse back-drives the wheels; F5 folds
that real rotation. Contribution ≈ −4…−8° per stop, roughly L/R symmetric.

**(B) ONE-WHEEL COLLAPSE (the killer):** rows where |L| and |R| differ wildly
at the END of a turn whose symmetry controller (K3) was alive the whole time:
- L45 row 2: L=−12, R=+4 — the RIGHT wheel only did 4 of its ~11 counts
- L45 row 5: L=−1 (!), R=+12 — the LEFT wheel did essentially NOTHING
- L45 row 1: 8/8 (both short)
- R90 row 3: 17/−16, coast=+4, got −64
The symmetry controller CANNOT fix this: it clamps at PWM_MIN=85 per wheel —
if a wheel's breakaway under load is above 85, that wheel stalls and no
correction exists. Row 5's left wheel moving 1 count for a 45° command is a
stall-like event (stiction/skid), possibly with a watchdog retry hidden in the
paste. Then coast/correction folds scatter got= by up to ±10° more
(row 5: stop diff ≥22.9 → final 13 → ~10 diff counts folded back).

### 45° verdict (guide checkpoint): FAILS as measured
Spread 9.8° >> the 3.88°/pair quantisation; worst case 25.2° = 44 % low.
But the failure mode is a DRIVE problem (one wheel not tracking), not the
resolution argument the report makes. The two clean rows (46.5 ×2, no coast)
show the machine hitting 46.5 = +1.5 E when BOTH wheels track — inside the
stage-4 §G band (+3.5…+5.5 near-miss, right order).

### L90 vs R90: means −8.2 vs +12.8 (|E|) → 4.6° apart as |E| — inside the
"within 2°"? NO — but the E signs straddle zero because mechanism (B) lands
randomly; gate judgement needs the protractor column (odometry alone cannot
separate body-vs-encoder truth given row 5).

### Decisive tests (bench, ~10 min)
1. **L45 ×3 protractor, robot untouched** (still the #1 open item): if the
   body is at ~46° while got= said 25.2 (row 5) → encoder undercount on the
   left wheel — encoder/tyre SLIP, not motion error.
2. **Motor-plug swap L↔R, repeat L45 ×3**: bad wheel follows the plug =
   motor/gearbox/wiring; stays left = chassis (caster load, bearing).
3. **Hand-spin each wheel by hand** (`x` bench mode on, watch `e`): rough
   gearbox or rubbing disc shows up as uneven resistance.
4. If (1) shows body≈got: reduce KICK_PWM 95→80, KICK_MS 60→40 (the kick's
   job is killing momentum, not travelling) and re-run the reps.

### NOT to do
- Do NOT retune trackMm/trackMmPivot from this data (bite is per-stop, E∝θ
  would be scale — the data contradicts scale).
- Do NOT raise PWM_MIN yet: first separate stalled-wheel vs kick-fold via
  tests 1-2; PWM_MIN=85 was floor-measured, the pack state today is unknown
  (volt not logged — standing note).

### Stage-4 §G gate-2 check (using these 5+5 reps as proxy 90° data)
E +4…+8 both dirs: L90 mean |E| 8.2 ✓ band-edge; R90 12.8 ✗ (mechanism B);
same sign both dirs: ✗ (straddle); σ ≤ 1.5: ✗ (7.5, 7.7); L/R within 2°:
✗ (4.6°). GATE NOT PASSED — but the failures trace to mechanism (B), which
the gate was never designed to test (it assumes healthy wheels). Fix the
wheel first, then the gate re-run is meaningful.
