# Stage 5 session 1 — full turn-accuracy matrix (verbatim + analysis, batch 3)

## Raw serial lines (verbatim)

L15:
```
tturn cmd=L15.0 got=7.8deg L=-4cnt R=0cnt coast=-2cnt
tturn cmd=L15.0 got=11.6deg L=-2cnt R=4cnt coast=1cnt
tturn cmd=L15.0 got=3.9deg L=2cnt R=4cnt coast=3cnt
tturn cmd=L15.0 got=15.5deg L=-4cnt R=4cnt
tturn cmd=L15.0 got=15.5deg L=-4cnt R=4cnt
```
R15:
```
tturn cmd=R15.0 got=-13.6deg L=6cnt R=-1cnt coast=1cnt
tturn cmd=R15.0 got=-3.9deg L=-2cnt R=-4cnt coast=-3cnt
tturn cmd=R15.0 got=-13.6deg L=5cnt R=-2cnt coast=1cnt
tturn cmd=R15.0 got=0.0deg L=-4cnt R=-4cnt coast=-4cnt
tturn cmd=R15.0 got=-15.5deg L=6cnt R=-2cnt coast=1cnt
tturn cmd=R15.0 got=-11.6deg L=4cnt R=-2cnt coast=1cnt
```
L30:
```
tturn cmd=L30.0 got=27.1deg L=-8cnt R=6cnt coast=-1cnt
tturn cmd=L30.0 got=23.3deg L=-8cnt R=4cnt coast=-2cnt
tturn cmd=L30.0 got=15.5deg L=0cnt R=8cnt coast=4cnt
tturn cmd=L30.0 got=31.0deg L=-8cnt R=8cnt
tturn cmd=L30.0 got=31.0deg L=-8cnt R=8cnt
```
R30:
```
tturn cmd=R30.0 got=-19.4deg L=4cnt R=-6cnt coast=-2cnt
tturn cmd=R30.0 got=-27.1deg L=7cnt R=-7cnt coast=-1cnt
tturn cmd=R30.0 got=-19.4deg L=10cnt R=0cnt coast=3cnt
tturn cmd=R30.0 got=-19.4deg L=4cnt R=-6cnt coast=-2cnt
tturn cmd=R30.0 got=-19.4deg L=3cnt R=-7cnt coast=-3cnt
```
L45 (third batch):
```
tturn cmd=L45.0 got=42.6deg L=-12cnt R=10cnt coast=-1cnt
tturn cmd=L45.0 got=36.8deg L=-7cnt R=12cnt coast=2cnt
tturn cmd=L45.0 got=36.8deg L=-7cnt R=12cnt coast=2cnt
tturn cmd=L45.0 got=34.9deg L=-12cnt R=6cnt coast=-3cnt
tturn cmd=L45.0 got=36.8deg L=-12cnt R=7cnt coast=-2cnt
tturn cmd=L45.0 got=36.8deg L=-10cnt R=9cnt coast=-1cnt
```
R45 (second batch):
```
tturn cmd=R45.0 got=-44.6deg L=13cnt R=-10cnt
tturn cmd=R45.0 got=-42.6deg L=14cnt R=-8cnt coast=1cnt
tturn cmd=R45.0 got=-40.7deg L=12cnt R=-9cnt coast=1cnt
tturn cmd=R45.0 got=-42.6deg L=11cnt R=-11cnt coast=-1cnt
tturn cmd=R45.0 got=-40.7deg L=11cnt R=-10cnt coast=-1cnt
tturn cmd=R45.0 got=-38.8deg L=8cnt R=-12cnt coast=-2cnt
```
L90 (second batch):
```
tturn cmd=L90.0 got=71.7deg L=-24cnt R=13cnt coast=-5cnt
tturn cmd=L90.0 got=83.4deg L=-21cnt R=22cnt coast=-1cnt
tturn cmd=L90.0 got=81.4deg L=-20cnt R=22cnt coast=2cnt
tturn cmd=L90.0 got=87.2deg L=-25cnt R=20cnt coast=-1cnt
tturn cmd=L90.0 got=89.2deg L=-24cnt R=22cnt coast=-1cnt
tturn cmd=L90.0 got=79.5deg L=-22cnt R=19cnt coast=-2cnt
```
R90 (second batch):
```
tturn cmd=R90.0 got=-79.5deg L=23cnt R=-18cnt coast=2cnt
tturn cmd=R90.0 got=-83.4deg L=21cnt R=-22cnt coast=-2cnt
tturn cmd=R90.0 got=-91.1deg L=23cnt R=-24cnt coast=-1cnt
tturn cmd=R90.0 got=-89.2deg L=24cnt R=-22cnt coast=1cnt
tturn cmd=R90.0 got=-91.1deg L=24cnt R=-23cnt coast=-1cnt
tturn cmd=R90.0 got=-89.2deg L=25cnt R=-21cnt coast=1cnt
```

## Full matrix (script-verified; F5 holds on all 45 rows)

| cell | n | mean got | mean E | σ | wheels<8cnt |
|---|---|---|---|---|---|
| L15 | 5 | 10.9 | −4.1 | 5.0 | 5/5 (quantisation: 15° = 3.9 cnt) |
| R15 | 6 | −9.7 | −5.3 | 6.3 | 6/6 (same) |
| L30 | 5 | 25.6 | −4.4 | 6.5 | 3/5 |
| R30 | 5 | −20.9 | −9.1 (|E|) | 3.4 | 5/5 |
| L45 | 6 | 37.4 | −7.6 | 2.6 | 4/6 |
| R45 | 6 | −41.7 | +3.3 | 2.0 | 0/6 |
| L90 | 6 | 82.1 | −7.9 | 6.2 | 0/6 |
| R90 | 6 | −87.2 | +2.8 | 4.7 | 0/6 |

45° cumulative (all batches): L45 n=16 mean ≈ 36.2 E −8.8 σ ~6; R45 n=9
mean ≈ −40.9 E +4.1 σ ~2.4.

## What changed with this batch

1. **90° cells improved markedly.** L90 got 89.2/87.2 in this batch; R90
   89.2–91.1 ×3 (E down to +0.9!). The R90 set is now essentially ON target
   with σ 4.7 — the earlier −64 outlier is gone. Whatever varies between
   batches (pack state, floor spot, warm gearbox) moves the collapse rate;
   it is not a fixed defect.
2. **R45 batch 2 is clean:** −38.8…−44.6, σ 2.0, no wheel under 8. The best
   cell of the whole session. L45 batch 3 σ 2.6 also tightened vs batch 2.
3. **15° confirms the resolution limit, as the guide predicts.** 15° = 3.9
   counts/wheel; got lands on the 3.88° grid (7.8/11.6/15.5 and 0.0/−15.5);
   R15 got 0.0 (−4/−4: wheels ticked OPPOSITE signs — landed one pair short)
   and L15 got 3.9 (one pair). Mean |E| 4–5° ≈ one paired step. This is the
   guide's "15° lies close to the resolution limit" live — expected, not a
   fault, and the pivot-mode workaround argument for the report writes itself.
4. **The collapse is still per-start random** (R30: L mags 4,7,10,4,3 —
   the left wheel collapses repeatedly for a RIGHT turn; R15 row 4 got 0.0).

## Session-wide turn conclusions (desk-side)

- **Odometry (F5): perfect.** 45/45 rows: got = (cR−cL)·k/b to 0.1°.
- **E is dominated by the random per-start wheel collapse + kick fold**, not
  by scale: E does NOT grow ∝θ (R45 +3.3 vs R90 +2.8 while θ doubles).
- **Best-case machine (both wheels tracking): E within ±1.5°, every angle**
  (46.5 at 45°, 89.2–91.1 at 90°). That is the machine the gate was meant to
  measure; the collapse is a separate, fixable defect.
- **σ at 45° spans 2.0–10.3 across batches** — batch-to-batch variance
  (pack/floor/thermal) is as large as the within-batch spread. σ≤1.5 gate
  clause unreachable until the collapse rate drops.

## Fix priority for the NEXT session (desk-ranked, needs the tel capture)

1. **Telemetry capture of a collapsing turn** (t on, L45, paste pwmL/pwmR):
   decides floor-clamp vs symmetry-fight. THE discriminator, still missing.
2. If floor-clamp: raise PWM_MIN 85→95 (row-9 precedent, pack-dependent) —
   5-minute sketch change, big expected win.
3. If symmetry-fight: freeze symGain at 0 for the first PULSE_MS+300 ms of
   each turn (let breakaway happen unequal, then equalise) — sketch change.
4. Kick-fold: KICK_PWM 95→80 / KICK_MS 60→40 — bounded ~2 cnt per stop,
   secondary to the collapse.
5. THEN re-run 45° ×5 + 90° ×5 both dirs for the stage-4 §G gate-2 verdict.

## Stage-4 checkpoint-4 accuracy-table status (these data feed it)

15/30/45/90 × L/R × 5 reps: cell counts now L15 5, R15 6, L30 5, R30 5,
L45 16, R45 9, L90 11, R90 11 — tables FILLABLE with real data (mean ± σ),
but the report must carry the collapse finding and the protractor-vs-got=
caveat: odometry self-report is exact, body angle unverified except where
the user protractors. 45° assignment requirement: NOT demonstrated (worst
13.6° = 70 % low); best-case runs demonstrate the machine CAN (46.5).
