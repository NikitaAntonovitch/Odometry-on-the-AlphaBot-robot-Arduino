# Stage 4 — accuracy table template (fill on the floor)

Status: TEMPLATE ONLY — every value is `<<unmeasured>>` until the robot
session writes it. No number in this file is evidence.

> **Session data 30.10.2026 (first pass — isolation battery, spin mode, one rep
> per angle, protractor not used):** the four rows marked with real console
> values below come from this session. R45 was not run — its rep-1 row reads
> `<<not yet run>>` (PENDING). Sign convention as printed by the firmware:
> got= is signed, + = CCW (left), − = CW (right); N_L/N_R signed per command.
> **Pivot continuation (same session):** L135 rep 1 with telemetry; the
> stationary wheel slipped 3 ticks (ASYMMETRY, noted in-row); three identical
> L90 pivotL reps (z pressed between them, operator-confirmed) flagged as a
> possible attractor/buffer artefact — kept verbatim, not spread. TRACK_PIVOT
> 145 remains provisional — no 360° multi-rev calibration this session.
> **Spin continuation (same power-on, z before every run):** L90 reps 2–4
> and R90 rep 2 added. L90 REPS 2–3 ANOMALY: `mode=spin` printed but the
> behaviour matches pivotL (left wheel 0/−2 ticks). NOT a confirmed
> mode-state leak: the go line prints the LIVE tmode (execTurn :923–928)
> and printed `spin` on all four; the left motor is electrically fine
> (R90 rep 2: L=28). Working theory: left-wheel breakaway failure inside
> spin mode after the pivot block; the direction switch (R90) preceded
> the perfect rep 4 (byte-identical to the isolation run). See
> NONDET_ANALYSIS.md.

Firmware under test: `lab1_stage6_autonomous.ino` (stage6-v1-FROZEN logic,
identical turn controller). k = 5.1087 mm/cnt, b_spin = 151.0 (confirmed),
b_pivot = 145.0 (PROVISIONAL — this session confirms or replaces it),
STOP_TOL_DEG = 0.7, PWM_CRUISE_TRN = 110.

## Pre-flight checklist (do ALL before the first turn)

- [ ] **Encoder hand-push:** `z` → push the robot straight ~0.5 m → `e`.
      L ≈ 98±10 cnt, R ≈ 98±10 cnt, R/L ratio 0.9…1.1. If one side reads
      near zero or half — STOP, fix the encoder before any accuracy run
      (stage-4 data is void on a miscounting wheel; rows 1-6 incident).
- [ ] **Voltage logged:** pack voltage before the block: ______ V,
      after the block: ______ V. No multimeter → write "not measured" and
      note the session's stall behaviour instead (3× start-stall = sagged pack,
      see stage-4 session row 9 precedent).
- [ ] **Constants from the calibration sheet:** K_MM 5.1087 ✓ / trackMm 151.0 ✓ /
      trackMmPivot 145.0 (provisional, under test) ✓ / PWM_MIN 85 ✓ —
      cross-checked against lab1_stage6 constants block, not re-typed.
- [ ] **TURN_STOP_LEAD_DEG recorded:** current value in the flashed sketch:
      ______ (expected 7.8). If a lead re-measure (Block C) changes it, write
      the new value here and mark which rows used which value.
- [ ] **Floor area:** 1440° spins sweep a full circle ×4 — clear ≥ 0.5 m radius.
      Protractor/paper mark at the start orientation; L=1 m tape for the
      45° tangent cross-bar check.
- [ ] Serial monitor 9600, line ending: something in the box (parser skips it).

## Column definitions (all tables)

| column | what to write |
|---|---|
| angle | commanded angle, deg (copy of target) |
| dir | `L` or `R` (direction from the command LETTER) |
| rep | 1…5 (run number within the cell) |
| target_deg | the number you typed, e.g. 15.0 |
| achieved_odometry_deg | the `got=` value from the `tturn` line, WITH its sign (got is signed: negative = clockwise/R) |
| measured_protractor_deg | deviation read off the protractor/paper mark at rest, signed in the same convention: CCW (left) +, CW (right) − |
| N_L / N_R | the `L=…cnt R=…cnt` counts from the same `tturn` line (signed) |
| coast_cnt | the `coast=` field of the `tturn` line |
| err_odom (deg) | computed later = achieved_odometry_deg − (dir==L ? +target : −target) — leave blank or let stage4_process.py fill |
| err_tape (deg) | computed later = measured_protractor_deg − (dir==L ? +target : −target) |

**Abort rule:** if the robot aborts (`err:` line, stall watchdog, fault hold)
the row is EVIDENCE, not garbage. Write `ABORT` in achieved_odometry_deg
(with the err line text in the notes column if present) and continue with the
next rep. Do NOT delete rows, do NOT re-run in place to replace one.
A cell with 3+ ABORTs out of 5 = stop the cell, note it, move on (likely pack
voltage — charge before continuing).

**Mode note:** spin = both wheels counter-rotate (default turn mode).
pivot = one wheel planted — only meaningful for the pivot sub-table; the
stage-4 controller applies the symmetry controller to spin only, by design.

## Table 1 — SPIN mode: 15/30/45/90 × L/R × 5 reps (40 rows)

> **Command per row:** for `dir`=L send `L<angle>` (e.g. `L15`), for `dir`=R
> send `R<angle>` (e.g. `R90`). Zero the encoders between reps is NOT needed
> (`z` optional; the tturn line is self-contained). One row = one command +
> one `tturn` line. Copy the `tturn` line verbatim into notes if anything
> looks odd.

| angle | dir | rep | target_deg | achieved_odometry_deg | measured_protractor_deg | N_L | N_R | coast_cnt | err_odom (deg) | err_tape (deg) |
|---|---|---|---|---|---|---|---|---|---|---|
| 15 | L | 1 | 15.0 | +15.5 | <<unmeasured>> | -3 | 5 | 2 | +0.5 | <<unmeasured>> |
| 15 | L | 2 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 3 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 4 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 5 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 1 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 2 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 3 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 4 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 5 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 1 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 2 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 3 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 4 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 5 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 1 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 2 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 3 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 4 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 5 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 1 | 45.0 | +40.7 | <<unmeasured>> | -4 | 17 | 1 | -4.3 | <<unmeasured>> |
| 45 | L | 2 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 3 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 4 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 5 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 1 | 45.0 | <<not yet run>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 2 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 3 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 4 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 5 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 90 | L | 1 | 90.0 | +89.2 | <<unmeasured>> | -22 | 24 | 2 | -0.8 | <<unmeasured>> |
| 90 | L | 2 | 90.0 | +85.3 | <<unmeasured>> | 0 | 44 | 1 | -4.7 | <<unmeasured>> |
| 90 | L | 3 | 90.0 | +87.2 | <<unmeasured>> | -2 | 43 | 2 | -2.8 | <<unmeasured>> |
| 90 | L | 4 | 90.0 | +89.2 | <<unmeasured>> | -22 | 24 | 2 | -0.8 | <<unmeasured>> |
| 90 | L | 5 | 90.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 90 | R | 1 | 90.0 | -89.2 | <<unmeasured>> | 26 | -20 | 2 | +0.8 | <<unmeasured>> |
| 90 | R | 2 | 90.0 | -87.2 | <<unmeasured>> | 28 | -17 | 2 | +2.8 | <<unmeasured>> |
| 90 | R | 3 | 90.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 90 | R | 4 | 90.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 90 | R | 5 | 90.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

**45° cross-check (tangent method, catches a scale error at the assignment's
gate angle):** after the 45° L cell, place the robot on the orientation mark,
run `L45` once, and measure the tangent offset of the wheel axis at a 1 m
reference line: expected lateral offset = 1000·tan(45°) = 1000 mm.
Sensitivity 34.9 mm/deg. Write the measured offset here: ______ mm.
(OUT-OF-SAMPLE flag: 45° was absent from the {90°, 1440°} calibration fit —
this row is the only independent check of track scale at 45°.)

## Table 2 — PIVOT mode: 15/30 required, 45 if time permits (× L/R × 5 reps)

> **Command per row:** send `m` first (turn mode → pivot) — confirm the mode
> line says pivot — then `L<angle>` / `R<angle>` as in table 1. After the
> pivot cell send `m` again to return to spin. In pivot mode ONE wheel is
> planted: the moving wheel measures the whole arc (1 count = k/b_pivot ≈
> 2.02°), so expect a FINER grid than spin, and no symmetry controller
> (by design — the planted wheel cannot be equalised).

| angle | dir | rep | target_deg | achieved_odometry_deg | measured_protractor_deg | N_L | N_R | coast_cnt | err_odom (deg) | err_tape (deg) |
|---|---|---|---|---|---|---|---|---|---|---|
| 15 | L | 1 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 2 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 3 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 4 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | L | 5 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 1 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 2 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 3 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 4 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 15 | R | 5 | 15.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 1 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 2 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 3 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 4 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | L | 5 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 1 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 2 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 3 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 4 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 30 | R | 5 | 30.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 1 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 2 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 3 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 4 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | L | 5 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 1 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 2 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 3 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 4 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |
| 45 | R | 5 | 45.0 | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> | <<unmeasured>> |

**360° pivot calibration run (confirms or replaces TRACK_PIVOT = 145):**
`m` → `L360` ×1, `R360` ×1. Record the two `tturn got=` values here:
L360 got= ______ , R360 got= ______ .
Track factor = mean(|got|)/360 × 145.0 → new b_pivot = ______ mm.
(If |mean E| ≤ 2° the 145 value is CONFIRMED; otherwise the computed value
becomes the new provisional and goes in the calibration audit note — do not
edit the .ino on the floor.)


### Off-plan pivot rows (session 30.10.2026 continuation — L90/L135, telemetry on)

> These angles are outside the 15/30/45 template grid but were run for the
> TRACK_PIVOT investigation. Kept here verbatim; `z` WAS pressed between the
> three identical L90 reps (operator-confirmed). Rows kept as measured — the
> identical reps 2–4 are flagged as a possible attractor/buffer artefact but
> NOT spread or deleted.

| angle | dir | rep | target_deg | achieved_odometry_deg | measured_protractor_deg | N_L | N_R | coast_cnt | err_odom (deg) | err_tape (deg) |
|---|---|---|---|---|---|---|---|---|---|---|
| 90 | L | 1 | 90.0 | +84.8 | <<unmeasured>> | 0 | 42 | 1 | -5.2 | <<unmeasured>> |
| 90 | L | 2 | 90.0 | +82.8 | <<unmeasured>> | 0 | 41 | 0 | -7.2 | <<unmeasured>> |
| 90 | L | 3 | 90.0 | +82.8 | <<unmeasured>> | 0 | 41 | 0 | -7.2 | <<unmeasured>> |
| 90 | L | 4 | 90.0 | +82.8 | <<unmeasured>> | 0 | 41 | 0 | -7.2 | <<unmeasured>> |
| 135 | L | 1 | 135.0 | +131.2 | <<unmeasured>> | 3 | 65 | 2 | -3.8 | <<unmeasured>> |

Note on L135 rep 1: ASYMMETRY — the "stationary" left wheel slipped 3 ticks
(~15 mm): not a true pivot. Systematic under-rotation −3.8°; see
L45_ANOMALY_ANALYSIS.md and the session root-cause notes.

## Protractor convention (agreed before the first row)

- Robot starts with its wheel axis on the mark, heading = 0.
- After the stop, read the angle of the robot's centreline against the mark.
- Sign: counter-clockwise (robot's left) = positive, clockwise = negative —
  matching the firmware's signed `got=` (R turns print negative got).
- Read to ±0.5° (paper protractor) or ±0.25° (printed 1 m tangent scale).

## Post-fill analysis

Run the processor (stdlib only; works on this file even when empty):

    python tools/stage4_process.py stage4_accuracy_template.md

It parses the tables above (skips `<<unmeasured>>`/`ABORT` rows), computes
per (mode, angle, dir): n, mean err_odom, spread (σ, n−1), mean err_tape —
and prints `INSUFFICIENT DATA` rows where nothing is filled yet. Verdict
rules it applies:

- 45° gate: worst-case |mean err_tape| at 45° must be ≤ 2.25° (5 % of 45°)
- mean error growing ∝ angle → track under-calibrated (scale)
- mean error constant, σ large → run-on / stop transient (control)
- σ not shrinking under any tuning → sensor resolution limit — document
