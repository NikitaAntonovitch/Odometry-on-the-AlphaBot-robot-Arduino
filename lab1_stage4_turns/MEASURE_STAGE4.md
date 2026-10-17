# Stage 4 — measurements to take at the lab (fill this in on the robot)

> **SESSION-1 protocol (17.10):** run order = §0 pre-flight → §1a track
> confirmation ×3/dir (supersedes §1) → §3/§4 accuracy tables → §7 tangent 45°.
> Paste every `tturn` line into the session log. §1 stays as history only.

Sketch to flash: `lab1_stage4_turns/lab1_stage4_turns.ino`. Serial: **9600**.
Commands: `L<deg>` / `R<deg>` turn left/right (T/B also accepted), `m` cycles
turn mode spin → pivotL → pivotR → spin, `J<mm>` sets the effective track
(for calibration), `K<gain>` symmetry gain, `t` telemetry, `e` encoders,
`c`/`b` stop, `h` menu.

Sign convention (adopted, recorded in the report): **positive angle = left =
counter-clockwise seen from above**. The command letter carries the direction;
a negative number is taken as its absolute value (same rule as F/B in stage 3).

Robot prep: charged cells, protractor or a marked circle on the floor (a sheet
of paper with radii drawn every 10° works), tape for the track estimate.
Battery voltage not measured (no multimeter — standing note).

## 0. Pre-flight gate (added 17.10 — stage-3 rows 1–6 + row 9 lessons)

Do NOT start the accuracy protocol until all four lines are checked:

- [ ] **Encoder health (hand-push):** `z` zero → push the robot straight
  forward ~0.5 m by hand → `e`. R/L ≈ 1.0 (accept 0.95…1.05). Rows 7–8 gave
  1.15/1.03 post-reseat — this is the formal confirmation pair; a formal
  FAIL voids every stage-4 number (rows 1–6 incident).
- [ ] **Pack:** freshly charged cells. Row 9: 3× start-stall at PWM_MIN 85
  on a sagged pack — breakaway rises above the floor under sag. If any
  turn aborts `err: stalled` with prog 0.0 → stop the session, charge,
  do not push through with J/K workarounds.
- [ ] **Greeting check:** on reset the sketch must print
  `stage4 v3: k=5.109 spin=151 pivot=145 [prov]` — confirms the flashed
  binary matches the constants under test.
- [ ] **Floor area:** 1440° spin sweeps a full circle ×4 — clear ≥ 0.5 m
  radius around the start mark; mark the start heading (tape line).

## 1. Track calibration by multi-revolution (guide p.24 — do this FIRST)

> **SUPERSEDED (17.10, STAGE4-AUDIT-1):** the sketch (stage4 v3) already carries
> `trackMm=151.0f` / `trackMmPivot=145.0f` from the 15.10 session + joint fit —
> the single L/R pair below and the stale `160.0 × f` base are obsolete. Run
> **§1a** instead; keep this section for the original derivation record.

`m` until `turn mode=spin`, then `L1440` (= 4 full revolutions; 5 recommended
by the guide, 4 keeps the robot inside a desk-scale area). When it stops, measure
the actual final heading vs the start mark (protractor or the marked circle).
Repeat `R1440`. The deviation spreads over 1440°, so 1° of reading error is
negligible. Correction: `b_calibrated = b_current × 1440 / actually_achieved`.

| dir | commanded | achieved (protractor) | correction factor | b_used (mm) |
|---|---|---|---|---|
| L1440 | 1440° | ______° | ______ | 160.0 × f |
| R1440 | 1440° | ______° | ______ | |

- after computing: `J<new track>` and re-run L1440 — achieved should now be ~1440
- difference L vs R correction sign → caster friction asymmetry (note it)
- tape measurement of the track: ______ mm; difference vs calibrated: ______
  (expected — contact point shift, slip, caster; explanation in report sec.3)

## 1a. Track CONFIRMATION at the fitted constants (§1 superseded — 17.10)

Sketch carries `trackMm=151.0 / trackMmPivot=145.0` (joint fit s=1.059,
run-on const +4.7°). This session does NOT re-derive b — it confirms or
refutes it per SIM_PREFILL §G gate 2: **E +4…+8 both dirs, same sign,
σ ≤ 1.5, L/R means within 2°**.

`m` until `turn mode=spin`. Verify greeting: `stage4 v3: k=5.109 spin=151
pivot=145 [prov]`. No J override — the flashed constants ARE the thing under
test. Between runs: re-place the robot on the start mark, let wheels settle.

| run | cmd | got= (deg) | achieved (protractor) | E = got − 1440 | protractor E |
|---|---|---|---|---|---|
| L1 | L1440 | | | | |
| L2 | L1440 | | | | |
| L3 | L1440 | | | | |
| R1 | R1440 | | | | |
| R2 | R1440 | | | | |
| R3 | R1440 | | | | |

- mean E(L) = ______  mean E(R) = ______  σ = ______
- decision: all four gate clauses pass → TRACK_MM 151.0 un-provisionalized
- any clause fails → do NOT edit the constant ad hoc; record and stop (the
  failure pattern decides: scale vs run-on vs asymmetry — bring it home)
- expected per the fit: E ≈ +5.8 (run-on only, scale removed); naive
  b = 151×1440/1445.8 = 150.4 was REJECTED by the [SIM] decomposition —
  do not re-introduce it

## 2. Angular resolution work-out (no robot needed — from the numbers)

With k = 5.1087 mm/count and the calibrated track b:
- spin: 1 count = 2k/b body-degrees = ______° (sim said ~0.46° at b=160)
- pivot: 1 count = k/b body-degrees = ______° (pivot resolves TWICE as finely)
- counts per wheel for 15° spin: ______ (sim: ~8) — vs STOP_TOL 0.7°: the
  tolerance is ~1.5 counts; the guide's "15° lies close to the resolution limit"
- counts per wheel for 30/45/90° spin: ______ / ______ / ______

## 3. Turn accuracy — spin mode (guide table: 15/30/45/90 × L/R × 5 reps)

For each cell: note the `got=` value from the `tturn` line (odometry truth) AND
the protractor reading. The table below takes 5 reps per cell; `got` and
protractor in separate columns so controller error and real-world error separate.

| angle | dir | rep | got (deg) | protractor (deg) |
|---|---|---|---|---|
| 15 | L | 1..5 | | |
| 15 | R | 1..5 | | |
| 30 | L | 1..5 | | |
| 30 | R | 1..5 | | |
| 45 | L | 1..5 | | |
| 45 | R | 1..5 | | |
| 90 | L | 1..5 | | |
| 90 | R | 1..5 | | |

(multiple rows per cell — leave space; mean and spread computed at the end)

## 4. Turn accuracy — pivot mode, at least the small angles

`m` → `turn mode=pivotL`. Same table for 15/30 (L only is fine, or both):

| angle | mode | rep | got | protractor |
|---|---|---|---|---|
| 15 | pivotL | 1..5 | | |
| 30 | pivotL | 1..5 | | |

Comparison of the two modes at 15° is THE report argument: pivot resolves
angles 2× better per count — if pivot's spread at 15° is visibly smaller,
the spin limitation is resolution, not control (report sec.5).

## 5. Diagnosis table (guide p.24) — fill after sec. 3

| observation | your verdict |
|---|---|
| mean error grows ∝ angle (track under-calibrated)? | |
| mean error constant (run-on — ramp/speed fix)? | |
| spread unchanged under any tuning (resolution limit)? | |

## 6. Enter into the sketch when done

```
static const float TRACK_MM = ___.0f;  // calibrated (sec. 1), not tape
```
(J<mm> is runtime — for the permanent value update TRACK_MM and reflash)

## 7. 45° tangent cross-bar method (independent 45° truth — added 17.10)

45° is OUT-OF-SAMPLE for the calibration fit (fit anchors were 90° and
1440°) — it needs its own geometric truth, not just the protractor circle.

Setup: tape a straight cross-bar (pencil/ruler) across the robot's centre
line. Start the robot with the bar parallel to a long tape measure laid on
the floor (L = 1.00 m from the pivot point). After a `L45`, the bar should
point at the 1 m mark. The MISS distance d on the tape converts to an angle:
**θ = atan(d / 1000) × 180/π** — sensitivities: 1 mm ≈ 0.057°, so
**17.5 mm ≈ 1°**. Reference chord distances at L = 1 m: 15° → 268 mm,
30° → 577 mm, 45° → 1000 mm (θ/2 geometry), each ±(18.7 / 23.3 / 34.9)
mm per degree — read d to ±5 mm ⇒ ±0.3°.

| rep | cmd | miss d (mm) | θ = atan(d/1000) | got= (deg) | got − θ |
|---|---|---|---|---|---|
| 1 | L45 | | | | |
| 2 | L45 | | | | |
| 3 | L45 | | | | |

- verdict: |got − θ| ≤ 2.25° on all reps → the 5% rule holds against
  INDEPENDENT geometry (not only odometry self-report)
- record whether θ lands above or below got= — a constant offset here is
  the same run-on signature the 1440 gate measures
