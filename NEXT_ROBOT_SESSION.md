# NEXT ROBOT SESSION — single executable checklist (~90 min on the floor)

Created 02.11.2026 (desk pass). Every value below marked `<<unmeasured>>` stays
unmeasured until this session fills it. No number here is evidence.

**Time budget:**

| block | content | time |
|---|---|---|
| A | pre-flight + stage-1 run-on matrix 2×2 | 15 min |
| B | stage-4 accuracy tables (spin 40 runs, pivot 15/30) | 30 min |
| C | TURN_STOP_LEAD_DEG re-measure per recipe | 15 min |
| D | stage-3 fill-ins (K6, lateral dev, telemetry) | 15 min |
| E | stage-6 acceptance — phase 2 ×3 + phase 3 | 15 min |

**Universal rules for every block:**
- An aborted run is EVIDENCE: record ABORT + the err line, continue. Never
  delete a row, never re-run in place to replace one.
- Do not edit constants on the floor (no `J<mm>` re-flashes mid-table); a
  value change gets its own confirm measurement.
- Templates to have open: `stage1_runon_matrix_template.md`,
  `stage4_accuracy_template.md`, `stage3_fillins_template.md`,
  `stage6_acceptance_template.md`.

---

## Block A (15 min) — pre-flight + stage-1 run-on matrix

**Flash:** `lab1_stage1_motor_test.ino` (bench sketch).

1. Pre-flight (fills the checklist in stage4_accuracy_template.md header):
   - `z` → hand-push straight ~0.5 m → `e`: L = ______ cnt, R = ______ cnt,
     R/L 0.9…1.1. FAIL ⇒ stop, fix encoder first.
   - Pack voltage: start ______ V, end ______ V (or "not measured" + stall
     behaviour note).
   - Confirm flashed constants against the calibration sheet: K_MM 5.1087,
     b_spin 151.0, b_pivot 145.0 (prov), PWM_MIN 85 — read the greeting line.
2. Run-on matrix (procedure in stage1_runon_matrix_template.md):
   - `+`/`-` to PWM 90 → drive across the kill line → `c` → tape → write
     coast_med cell. Repeat with `b` → brake_med cell.
   - PWM 170 → same two presses → coast_high / brake_high cells.
   - Fill both rows (distance mm + stopwatch time ms) of the template matrix.

**Commands:** `h` (menu) `+` `-` `1`/`2` `c` `b` `e` `z`.

**Acceptance:** all 4 cells (×2 rows) filled with tape numbers + voltage
logged ⇒ block done. Outcomes feed: checkpoint-1 item 6 closes (◑→☑) and the
stage-3 DEC_MM re-tune flag.

---

## Block B (30 min) — stage-4 accuracy tables

**Flash:** `lab1_stage6_autonomous.ino` (frozen logic; turn controller
identical to stage 4).

1. SPIN table (40 rows of stage4_accuracy_template.md §Table 1):
   - Per row: place robot on the orientation mark, send `L15` / `R15` /
     `L30` / … / `R90`, copy `tturn got=… L=… R=… coast=…` into the row,
     read the protractor against the mark, write it in measured_protractor_deg.
   - Order that saves walking: L15×5, R15×5, L30×5, R30×5, L45×5, R45×5,
     L90×5, R90×5. Reset placement to the mark EVERY run (spin returns the
     chassis centre but the mark is the heading reference).
   - 45° tangent cross-check once after the L45 cell (template §Table 1
     footer): offset at the 1 m line = ______ mm.
2. PIVOT table (template §Table 2): `m` (mode → pivot, confirm the line) →
   15/30 × L/R × 5 reps, same per-row drill → `m` back to spin.
3. 360° pivot calibration runs: `m` → `L360` ×1, `R360` ×1 → write both
   got= values in the template §Table 2 footer → compute new b_pivot factor
   there (do not edit the .ino on the floor).

**Commands:** `L<deg>` `R<deg>` `m` `e` `z` (z optional between reps).

**Acceptance:** 40/40 spin rows + 20/20 pivot rows (45-pivot row optional)
each carry either numbers or an honest ABORT ⇒ block done. Processing:
`python tools/stage4_process.py stage4_accuracy_template.md` — INSUFFICIENT
DATA rows tell you what is still missing; the 45° 5%-gate verdict prints at
the bottom. The grading requirement "45° achievable" is decided by this block.

---

## Block C (15 min) — TURN_STOP_LEAD_DEG re-measure (recipe)

**Flash:** `lab1_stage7_faultrecovery.ino` — the constant lives ONLY here
(stages 4/5/6 stop turns by kick+coast with allowance = STOP_TOL_DEG; the
stop-early lead is a stage-7 mechanism).

**Current value (lab1_stage7_faultrecovery.ino:73):** `TURN_STOP_LEAD_DEG = 7.8f`
— comment: stop this much EARLY: brake residual + stop-step overshoot at the
PWM_MIN floor (measured as got= minus target with lead=0; sim brake plant:
2 count-pairs = 7.75 deg). Terse in-file recipe: "flash with 0.0, run L90,
set lead = (got − target), re-flash. quantisation floor: 1 pair = 3.9 deg."
**The precise recipe below is the authoritative one** (the in-source comment
is a summary; do not change the value on the floor).

Procedure (fills the slot in stage4_accuracy_template.md pre-flight):
1. This recipe requires lead temporarily 0.0 — that is a ONE-LINE re-flash of
   the constant BEFORE the block (allowed: it is the measurement itself, not
   a tuning change). Flash, confirm greeting.
2. Run `L90` ×5 and `R90` ×5 (same drill as Block B rows). Copy every
   `tturn got=` value: L: ______ ×5, R: ______ ×5.
   - Discard NOTHING, but ANNOTATE collapse turns (wheels short of ~23 cnt
     total): the mean for the recipe uses HEALTHY runs only (both wheels
     broke away), because the lead is a property of the brake plant, not of
     per-start collapse. Healthy-run identification: per-wheel counts ≈
     23±2 for L90.
3. mean_got_healthy = ______ deg. **lead_new = mean_got_healthy − 90** (L;
   for R: |mean_got| − 90). If |lead_new| < 3.9, set lead_new = 3.9 (one
   count-pair — the quantisation floor from the comment; smaller leads
   cannot be resolved).
4. Re-flash with the ORIGINAL 7.8 first if lead_new within ±1.9° of it (same
   grid cell) — then the value stands and no further action. Otherwise the
   measured value goes into the report + the audit note; the .ino edit is
   made at the desk AFTER the session, with its own confirm run
   (`L90` ×3, mean |E| ≤ 1 pair).

**Acceptance:** 10 tturn lines copied, lead_new computed and compared to 7.8
with the ±1.9° grid rule ⇒ block done.

---

## Block D (15 min) — stage-3 fill-ins

**Flash:** `lab1_stage3_straight.ino` (wheel-guard build).

1. K6 rows (stage3_fillins_template.md §b): `K6` (confirm line) → `B500` ×3
   (×5 if time) → copy L−R end counts ×5.1087 into the cells + symptom line
   (weaving / stutter / none).
2. Lateral deviation K0 vs K3 (§c): straight lane + tape line. `K0` →
   `F500` ×5, measure lateral deviation at stop (mm, left +) → fill K0
   table. `K3` → `F500` ×5 → fill K3 table.
3. Telemetry (§d) — only if the K6 symptoms need the time series to be
   conclusive: telemetry build (`-DTELEMETRY_TURN=1`), `K3` → `F500` capture
   → `K6` → `F500` capture; plot stub `stage3_tel_plot.py` usage is in the
   template. Skip if K6/§c rows already tell the story — the checkpoint
   item then closes on the end-state numbers + symptom lines.
4. Ramp/run-on section (§a) and controller-vs-scale table (§e): each needs a
   tape at every run — fold these INTO the §c runs where possible (same F500
   runs give both lateral deviation AND overshoot AND the three §e numbers
   if you tape start→stop distance and read got=).

**Commands:** `K<gain>` `F<mm>` `B<mm>` `e` `h`.

**Acceptance:** K6 symptom recorded + 5×5 lateral rows filled (or ABORT
notes) ⇒ checkpoint-3 pending items have their evidence. §a/§e can carry
over to a second short session if the tape work overruns — mark what moved.

---

## Block E (15 min) — stage-6 acceptance (THE grading gate)

**Flash:** `lab1_stage6_autonomous.ino` (FROZEN — fault layer is stage 7's
sketch; the acceptance runs on the frozen source).

1. Re-mark A (wheel-axis cross + orientation mark) and B (tape cross at the
   predicted end; B-zone ±15 cm) — stage6_acceptance_template.md phase 1.
2. Phase 2 ×3 (template table): place on A + mark → paste the whole string
   `G0.8 L90 G1.2 R45 G0.6` as ONE line → hands off until the 5th result
   line → measure end x/y offsets from the B cross + heading error → fill
   the row, paste the console verbatim under the table. Carry back to A,
   repeat.
3. Tape the actual path length once: ______ mm (odometry route = Σ got).
4. Phase 3: `python tools/stage6_process.py --md stage6_acceptance_template.md
   --odo <odo_mm> --tape <tape_mm>` → paste the summary block into the
   template's phase-3 table.

**Abort/anomaly:** collapse on R45 (E ≥ 4.7°) or out-of-zone finish ⇒ row
stays, run-card gate triggers the breakaway-patch path (patch is on file,
NOT applied to the frozen source). Robot does not move ⇒ power switch /
cells first, then it is an abort row.

**Acceptance:** 3 filled rows + the phase-3 summary pasted ⇒ the REPORT_STAGE6
§3 table and §4 fill from it; per the run-card: 3/3 in B-zone and no turn off
≥ 4.7° → FREEZE stands.

---

## After the session (desk)

- Fill the four templates into their reports (stage1 §5 ◑→☑, stage4 §5
  checkpoint table, stage3 §5.8 PENDING→SATISFIED rows, stage6 §3/§4).
- Run `tools/stage4_process.py` + `tools/stage6_process.py` final versions
  and paste their blocks.
- Decide the two ASK items from CALIBRATION_AUDIT.md: dead-band 55-vs-85
  provenance note (or a Block-A dead-band re-measure), and the comment-lag
  edits (PROVISIONAL wording, REPORT_STAGE4 blank track line).
- Commit: reports + templates-with-data as separate logical commits.
