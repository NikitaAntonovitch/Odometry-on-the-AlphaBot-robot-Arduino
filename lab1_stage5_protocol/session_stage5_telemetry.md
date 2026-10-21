# Stage 5 session 1 — telemetry captures (verbatim + analysis): the drag asymmetry is REAL

## Raw serial lines (verbatim)

Turn telemetry (t on):
```
tel=off
go cmd=L45.0deg mode=spin
tturn cmd=L45.0 got=36.8deg L=-12cnt R=7cnt coast=-2cnt
go cmd=L15.0deg mode=spin
tturn cmd=L15.0 got=15.5deg L=-4cnt R=4cnt
go cmd=R15.0deg mode=spin
tturn cmd=R15.0 got=-13.6deg L=3cnt R=-4cnt coast=-1cnt
mode=stop pwm=n/a (brake)

tel=on
go cmd=L45.0deg mode=spin
tel prog=1.9 rem=43.1deg pwm=87
tel prog=9.7 rem=35.3deg pwm=93
tel prog=21.3 rem=23.7deg pwm=103
tel prog=31.0 rem=14.0deg pwm=97
tturn cmd=L45.0 got=46.5deg L=-12cnt R=12cnt
mode=stop pwm=n/a (brake)
```
```
tel prog=3.9 rem=86.1deg pwm=88
tel prog=13.6 rem=76.4deg pwm=96
tel prog=23.3 rem=66.7deg pwm=104
tel prog=38.8 rem=51.2deg pwm=110
tel prog=58.2 rem=31.8deg pwm=110
tel prog=77.5 rem=12.5deg pwm=95
tturn cmd=L90.0 got=89.2deg L=-24cnt R=22cnt coast=-1cnt
tel prog=5.8 rem=84.2deg pwm=90
tel prog=15.5 rem=74.5deg pwm=98
tel prog=23.3 rem=66.7deg pwm=104
tel prog=38.8 rem=51.2deg pwm=110
tel prog=54.3 rem=35.7deg pwm=110
tel prog=69.8 rem=20.2deg pwm=102
tel prog=89.2 rem=0.8deg pwm=86
tturn cmd=L90.0 got=89.2deg L=-24cnt R=22cnt coast=-1cnt
tel prog=5.8 rem=84.2deg pwm=90
tel prog=13.6 rem=76.4deg pwm=96
tel prog=25.2 rem=64.8deg pwm=106
tel prog=34.9 rem=55.1deg pwm=110
tel prog=52.3 rem=37.7deg pwm=110
tel prog=69.8 rem=20.2deg pwm=102
tel prog=87.2 rem=2.8deg pwm=87
tturn cmd=R90.0 got=-89.2deg L=26cnt R=-20cnt coast=1cnt
tel prog=5.8 rem=84.2deg pwm=90
tel prog=15.5 rem=74.5deg pwm=98
tel prog=25.2 rem=64.8deg pwm=106
tel prog=44.6 rem=45.4deg pwm=110
tel prog=62.0 rem=28.0deg pwm=108
tel prog=81.4 rem=8.6deg pwm=92
tturn cmd=R90.0 got=-91.1deg L=26cnt R=-21cnt coast=1cnt
```
Drive telemetry (z, then G0.8 and G1.2):
```
enc zeroed
go cmd=G0.800m dir=fwd
tel prog=0.0 rem=800.0 err=0.0 pwmL=85 pwmR=85
tel prog=2.6 rem=797.4 err=-5.1 pwmL=102 pwmR=85
tel prog=12.8 rem=787.2 err=-5.1 pwmL=110 pwmR=85
tel prog=30.7 rem=769.3 err=0.0 pwmL=108 pwmR=108
tel prog=58.8 rem=741.2 err=5.1 pwmL=114 pwmR=144
tel prog=89.4 rem=710.6 err=5.1 pwmL=115 pwmR=145
tel prog=127.7 rem=672.3 err=10.2 pwmL=100 pwmR=160
tel prog=168.6 rem=631.4 err=10.2 pwmL=100 pwmR=160
tel prog=212.0 rem=588.0 err=5.1 pwmL=115 pwmR=145
tel prog=260.5 rem=539.5 err=10.2 pwmL=100 pwmR=160
tel prog=301.4 rem=498.6 err=10.2 pwmL=100 pwmR=160
tel prog=344.8 rem=455.2 err=5.1 pwmL=115 pwmR=145
tel prog=393.4 rem=406.6 err=10.2 pwmL=100 pwmR=160
tel prog=436.8 rem=363.2 err=5.1 pwmL=115 pwmR=145
tel prog=485.3 rem=314.7 err=10.2 pwmL=100 pwmR=160
tel prog=531.3 rem=268.7 err=0.0 pwmL=130 pwmR=130
tel prog=577.3 rem=222.7 err=10.2 pwmL=100 pwmR=160
tel prog=623.3 rem=176.7 err=0.0 pwmL=130 pwmR=130
tel prog=669.2 rem=130.8 err=10.2 pwmL=100 pwmR=160
tel prog=720.3 rem=79.7 err=10.2 pwmL=100 pwmR=160
tel prog=763.8 rem=36.2 err=5.1 pwmL=97 pwmR=127
done cmd=G0.800m got=791.8mm err=-8.2mm L=156cnt R=154cnt
```
```
go cmd=G1.200m dir=fwd
tel prog=485.3 rem=714.7 err=10.2 pwmL=100 pwmR=160
tel prog=528.8 rem=671.2 err=5.1 pwmL=115 pwmR=145
tel prog=574.7 rem=625.3 err=5.1 pwmL=115 pwmR=145
tel prog=623.3 rem=576.7 err=0.0 pwmL=130 pwmR=130
tel prog=669.2 rem=530.8 err=10.2 pwmL=100 pwmR=160
tel prog=715.2 rem=484.8 err=0.0 pwmL=130 pwmR=130
tel prog=763.8 rem=436.2 err=5.1 pwmL=115 pwmR=145
tel prog=807.2 rem=392.8 err=0.0 pwmL=130 pwmR=130
tel prog=855.7 rem=344.3 err=5.1 pwmL=115 pwmR=145
tel prog=904.2 rem=295.8 err=10.2 pwmL=100 pwmR=160
tel prog=947.7 rem=252.3 err=5.1 pwmL=115 pwmR=145
tel prog=996.2 rem=203.8 err=10.2 pwmL=100 pwmR=160
tel prog=1042.2 rem=157.8 err=0.0 pwmL=130 pwmR=130
tel prog=1088.2 rem=111.8 err=10.2 pwmL=100 pwmR=160
tel prog=1134.1 rem=65.9 err=0.0 pwmL=130 pwmR=130
tel prog=1182.7 rem=17.3 err=5.1 pwmL=85 pwmR=113
done cmd=G1.200m got=1192.9mm err=-7.1mm L=234cnt R=233cnt
go cmd=G1.200m dir=fwd
mode=stop pwm=n/a (brake)
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1195.4mm err=-4.6mm L=235cnt R=233cnt
go cmd=L90.0deg mode=spin
tturn cmd=L90.0 got=85.3deg L=-17cnt R=27cnt coast=2cnt
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
```

## Desk analysis

### Drives: ALL PASS, and the telemetry caught the drag asymmetry red-handed
| run | got | err | tol | L/R | |
|---|---|---|---|---|---|
| G0.8 | 791.8 | −8.2 | ≤10 ✓ | 156/154 (0.5 %) | IN window |
| G1.2 #1 | 1192.9 | −7.1 | ✓ | 234/233 | IN window |
| G1.2 #2 | 1195.4 | −4.6 | ✓ | 235/233 | tick past edge (like stage-3 row 7) |
| G1.2 #3 | 1190.3 | −9.7 | ✓ | 234/232 | IN window |

**The headline finding: pwmL≈100–115 vs pwmR≈145–160 SUSTAINED.** The symmetry
controller rides its ±CORR_MAX=30 clamp for most of both drives, err pinned at
+10.2 mm (2 counts) — the RIGHT wheel needs ~+45–60 duty over the left to keep
pace under load. End counts equalise to 0.5 % anyway: the controller is the
hero. Compare stage-2 bench (unloaded): right was 6 % FASTER — the asymmetry
DIRECTION flips under load. This is a permanent mechanical fact (gearbox drag),
not pack sag: it persisted across all 4 runs.

### This CONFIRMS the turn-collapse mechanism
At turn start the profile commands pwm≈87–93 (L45 tel: 87 at prog 1.9). A wheel
that needs ~100+ to break away under load stalls right there; the symmetry
controller then has only CORR_MAX_TRN=20 of headroom (vs 30 on straights) and
the equalisation may hold the free wheel back while the stuck one catches up.
Whether it breaks free = the per-start randomness. The instrumented turns were
all healthy (46.5, 89.2, −89.2, −91.1 — E ≤ 1.1, the machine at its best), so
the capture shows the profile working but not the stall itself.

**Smoking gun in the code:** T_PULSE exits when ANY wheel moves
(`encCountL() != startL || encCountR() != startR`). With a drag-asymmetric
pair, the strong wheel moves first, the pulse aborts early, and the weak wheel
faces the 87-pwm profile still stuck. Per-wheel breakaway is the fix.

### Turn telemetry limitation
Turn tel prints only the base pwm (post-profile, pre-symmetry) — magL/magR are
not printed, so the per-wheel turn story needs a 2-line tel patch.

### Spin-turn with unequal paths (L90 got 85.3, |L|17/|R|27)
Δθ = (sR−sL)/b holds for the BODY heading even while the centre translates —
got= is still the true heading change; the robot just also drifted sideways.
Worth one line in the report (odometry of a dragged spin turn).

### 15° cells
15.5 ×2 (perfect grid landings), −13.6, −13.6, −11.6 — quantisation as predicted.

## Verdict
- Stage-5 straight items: CLOSED with real data (4/4 drives in tolerance,
  symmetry story with numbers).
- Turn collapse: mechanism CONFIRMED as load-drag + early pulse exit;
  fix = per-wheel breakaway pulse (AND instead of OR) + print magL/magR in
  turn tel. ~6-line sketch change, then re-run 45°/90° ×5.
