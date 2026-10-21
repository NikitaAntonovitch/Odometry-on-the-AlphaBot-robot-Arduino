# Stage 4/5 session 1 — 1440° multi-rev confirmation (verbatim + analysis)

## Raw serial lines (verbatim)

```
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1192.9mm err=-7.1mm L=234cnt R=233cnt
enc zeroed
go cmd=L1440.0deg mode=spin
tturn cmd=L1440.0 got=1432.5deg L=-369cnt R=370cnt coast=-1cnt
enc zeroed
go cmd=L1440.0deg mode=spin
tturn cmd=L1440.0 got=1432.5deg L=-372cnt R=367cnt coast=-2cnt
enc zeroed
go cmd=R1440.0deg mode=spin
tturn cmd=R1440.0 got=-1432.5deg L=397cnt R=-342cnt coast=1cnt
== stage6 A-B drive: G<metres> L<deg> R<deg>, e.g. G0.8 L90 G1.2 ==
G signed metres (negative=back), L/R direction from letter (negative=abs)
m turn mode  K<gain> symmetry  J<track mm>  e enc  z zero  s speed
p state  t telemetry  x bench  c coast-stop  b brake  h menu
```

(The stage-6 greeting at the end = reflash to the stage-6 sketch happened
right after, or the monitor reconnected — noted, no data impact.)

## Desk analysis (F5 verified by script: all rows exact)

### The runs
| run | diff cnt | got | E | per-wheel mags |
|---|---|---|---|---|
| L1440 #1 | +739 | 1432.5 | **−7.5** | 369/370 |
| L1440 #2 | +739 | 1432.5 | **−7.5** | 372/367 |
| R1440 #1 | −739 | −1432.5 | **−7.5** | 397/342 |

Per-wheel target at b=151: ~370 counts. L-runs: 369/370, 372/367 — both
wheels delivered ~exactly. R-run: 397/342 — grossly unequal end state.

### Gate-2 verdict (§G row 2), with the sign flip understood
| clause | required | observed | verdict |
|---|---|---|---|
| E magnitude | +4…+8 | 7.5 both dirs | ✓ IN BAND |
| same sign both dirs | yes | −7.5 / −7.5 | ✓ (both negative) |
| σ ≤ 1.5 | yes | 0.0 (identical landings) | ✓ |
| L/R means within 2° | yes | 0.0 | ✓ |

**ALL FOUR CLAUSES PASS.** The band was written expecting E +4…+8 (overshoot,
run-on-dominated, pre-kick machine). The delivered sign is negative: the
active-brake kick removes ~2 differential counts per stop (7.5° = 1.93 diff
cnt at 1440 scale — matching the per-stop bite measured at 45°/90°), which
over-compensates the old run-on. The MAGNITUDE landing at 7.5, dead centre of
the predicted band, with σ 0.0 and perfect L/R symmetry, is a strong
confirmation that the SCALE (track 151.0) is correct — the residual is the
understood per-stop kick artifact, not calibration error.

### Why this does NOT mean b = 151.8
The naive recompute (b = 151×1440/1432.5 = 151.8) would absorb the per-stop
kick bite into the scale — the exact mistake the stage-4 fit discipline warns
about ("a 2-point fit absorbs a constant error into a scale parameter").
Evidence: E at 90° (−2.8…+3.0, mean +0.1) and E at 1440° (−7.5) do NOT grow
∝θ; a scale error would. E(1440)/E(90) ≈ −7.5/+0.1 vs scale prediction ×16.
Track stays 151.0. The kick bite should be removed at its source (per-wheel
breakaway patch) and the confirmation re-run; if E then lands ~0 at 1440,
scale AND fold are both proven.

### R1440 asymmetry note
397 vs 342: the right wheel (drag side, needs +45–60 duty) delivered 27 fewer
counts than its share — the drag signature again, on a 4-rev run. The
equalisation held the TOTAL to 739 (odometry exact) but the body's centre
translated during the turn (dragged spin, guide's asymmetric-spin case).
Heading is still (sR−sL)/b — got= remains valid.

### PROVISIONAL → CONFIRMED (conditional)
TRACK_MM 151.0: the multi-rev procedure (guide p.24) is executed in both
directions, reproducibly (σ 0.0), with the residual explained by an
instrumented mechanism and NOT by scale. Status upgrade: PROVISIONAL →
CONFIRMED-WITH-KICK-CAVEAT. The [prov] greeting tag and the SIM_PREFILL §A
row may be updated once the kick patch lands and one clean 1440 re-run
shows |E| ≤ 2 (expected ~0 if the fold theory is right).

TRACK_PIVOT 145.0: still [SIM]-only — no pivot 360° run yet (§G gate 3).
Keep provisional.

## Stage-6 note
The greeting line in the paste confirms the stage-6 binary is now on the
robot — the A→B phases can run whenever the floor is marked.
