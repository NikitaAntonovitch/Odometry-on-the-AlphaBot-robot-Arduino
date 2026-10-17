# STAGE4-PREP-1 — MEASURE_STAGE4 session-readiness patch (17.10, desk)

**Scope:** stage-4 worksheet only, per STAGE4-AUDIT-1 findings 1/4/8 ("must
be added to MEASURE_STAGE4 before the session"). No sketch changes, no data.

1. **§0 pre-flight gate (NEW):** encoder hand-push R/L≈1.0 formal pair
   (rows 7-8 gave 1.15/1.03 — this closes the informal status), fresh pack
   (row 9: 3× start-stall at PWM_MIN 85 under sag — abort, don't push
   through), greeting check (`stage4 v3: k=5.109 spin=151 pivot=145 [prov]`
   = flashed binary matches constants under test), floor area + start mark.
2. **§1a confirmation protocol (NEW):** 1440° ×3/dir table (got= +
   protractor columns), gate clauses from SIM_PREFILL §G row 2 verbatim
   (E +4…+8 both dirs, same sign, σ ≤ 1.5, L/R within 2°), expected E ≈
   +5.8 (run-on only), naive b=150.4 explicitly barred from re-introduction,
   fail path = record-and-stop (no ad-hoc constant edits).
3. **§1 SUPERSEDED banner:** stale b=160 base and single L/R pair marked
   obsolete — history kept for the derivation record.
4. **§7 tangent cross-bar method (NEW):** 45° is OUT-OF-SAMPLE for the
   {90°,1440°} fit; independent geometry truth at L=1 m (17.5 mm ≈ 1°,
   reference chords 268/577/1000 mm for 15/30/45°, sensitivities
   18.7/23.3/34.9 mm/deg), 3-rep table, verdict |got − θ| ≤ 2.25°, offset
   direction read as the run-on signature.

**Not touched:** the .ino, REPORT_STAGE4.md blanks (they fill AFTER the run,
from §1a/§7 data — not pre-filled), SIM_PREFILL (gates quoted, not edited).

# DATA-XFER-1 — Stage 3 v4 session data entry (17.10, desk)

## Task ID: PRED-XFER-1 — model-prediction section + session rows 7-9 (17.10, desk)

**Scope:** append real session rows 7-9, add the PRED section 5.4a for the
pending K sweep, close exactly one TODO (K6 got=). No sketch changes, no
fabrication.

**session.csv:** rows 7-9 appended (REAL transcriptions):
- row 7: TestD F300 K6, got=296.3, R/L=1.15 — encoder apparently fixed pre-run
- row 8: TestD F500 K6, got=493.0, R/L=1.03 — closes the K6-got TODO;
  settled +505.7 mm (97/101 cnt)
- row 9: K0 F500 — 3x start-stall abort (pwm 85/85, prog 0.0 for 1500 ms,
  watchdog); pack sag suspected; no got, no counts

**stage3_tables.py:** unchanged; re-run on the 9-row session —
- rows 1-6 still FAULT-excluded (excluded list unchanged)
- row 7: err=-3.7 -> OUTSIDE [-10,-6] window (rem 3.7 on tick grid - stop fired
  in the same step past the edge; consistent, not a bug)
- row 8: err=-7.0 -> IN window ✓; R/L=1.03
- row 9: empty got/counts -> skipped by the existing guards, no crash
- aggregates: 3 clean rows eligible (7, 8 + none other with tape) - none carry
  tape, so Tables 2/4 remain TODO

**Report (S5_ASSEMBLED):** new subsection **5.4a "Model predictions for the
symmetry-gain sweep (PRED)"** inserted between 5.4 and 5.5 - constants block,
Table P1 (parameter-free), Table P2 (parametric g sweep), anchor paragraph on
the row-8 residual (-15.3 mm, 25-70x above PRED; hypotheses (a) corr-not-
applied vs (b) ~3% left-encoder bias; telemetry sweep decides). Limitations:
"Status update (same session)" paragraph appended verbatim.

**TODO inventory:** K6-got item CLOSED (row 8); everything else open,
unchanged. PRED values confined to section 5.4a - session.csv and all
measurement cells untouched by them.

# DATA-XFER-1 — Stage 3 v4 session data entry (17.10, desk)

## Task ID: DATA-XFER-1

**Scope:** data entry + honest flagging only. No sketch changes, no data
fabrication, no fault diagnosis (status stays open).

## What was done

### 1. session.csv (created — no earlier partial entry existed)
- header: `run,gain,dir,cmd,got,L_cnt,R_cnt,odo_L,odo_R,tape_x,tape_y,volt,notes`
- 6 rows appended (canonical block from the task, verbatim transcriptions of
  the serial console): runs 1–6, **all tagged FAULT:L/R**
- tape_x / tape_y / volt left empty in every row (postponed until fault resolved)
- run 1 `got=TODO` (done line not captured; recoverable from serial scrollback)
- no older-session rows modified (session.csv did not exist before this task)

### 2. stage3_tables.py (created)
- reads session.csv; **FAULT:L/R rows excluded from ALL mean/dispersion
  aggregates** (Tables 2 and 4), excluded-rows list printed explicitly on
  every run
- got-window check unchanged (`err in [-10,-6]` fwd / `[+6,+10]` back);
  |scale| ≤ 8.5 mm k-consistency check unchanged
- one fix during bring-up: `got=TODO` rows skip the numeric window check with
  a printed note instead of crashing (no value was invented to make it run)

### 3. Script run output (excerpt — full output in the transcript)
```
rows total: 6   REAL: 6   clean (aggregate-eligible): 0   FAULT-excluded: 6

-- excluded rows (FAULT:L/R, kept out of ALL aggregates) --
  run 1: gain=6   dir=fwd cmd=500 got=TODO  L=75  R=118 R/L=1.57
  run 2: gain=0   dir=fwd cmd=500 got=490.4 L=68  R=124 R/L=1.82
  run 3: gain=man dir=fwd cmd=-   got=TODO  L=128 R=200 R/L=1.56
  run 4: gain=man dir=fwd cmd=-   got=TODO  L=81  R=137 R/L=1.69
  run 5: gain=man dir=fwd cmd=-   got=TODO  L=72  R=128 R/L=1.78
  run 6: gain=man dir=fwd cmd=-   got=TODO  L=71  R=129 R/L=1.82

-- per-row checks --
  run 2: got-window check err=-9.6 mm -> IN [490,495] window [FAULT - not aggregated]
  R/L per row: 1.57 / 1.82 / 1.56 / 1.69 / 1.78 / 1.82  [all FAULT]

-- Table 2: TODO (no tape measurements)
-- Table 4: TODO (no clean got+tape pairs)
-- Table 1: K-effectiveness unresolved under fault; K6 R/L 1.57 with control
   vs 1.56 without -> no measurable correction visible; T3 telemetry re-run pending

-- summary --
  aggregates computed over: 0 clean rows
  excluded: 6 FAULT rows
  discipline: no FAULT row entered any mean/spread; no value estimated or smoothed
```

## Cells filled vs flagged vs still-TODO

**Filled (REAL, mapped unambiguously, single-run evidence — no aggregation):**
- Table 4, run 2 err_ctl = **−9.6 mm → IN [−10, −6] window** (got=490.4 on cmd 500)
- decel profile evidence: **pwm 123 @ rem 50.4, pwm 106 @ rem 27.4, matching
  85 + 45·(rem/60)** (telemetry, run 2)
- coast run-on: **+7.7 mm** after the v4 settled-odom patch (was ~+45 mm
  pre-patch, run 2 settled 498.1 vs got 490.4)
- R/L split per run: 1.57 / 1.82 / 1.56 / 1.69 / 1.78 / 1.82 (fault signature)
- K6 observation: R/L 1.57 with control vs 1.56 without → no measurable
  correction visible under the fault

**Marked TODO(FAULT)** — depends on the unresolved fault:
- all K-gain effectiveness cells (Tables 1 aggregation), symmetry metrics,
  straightness / lateral deviation (Table 2), tape-based scale error (Table 4)

**Still TODO (unchanged):**
- run 1 got= (serial scrollback), all tape_x/tape_y, all volt, the 4th gain,
  telemetry re-runs (T3), hand-push discriminator test

## Open TODO inventory (updated PRED-XFER-1)

**CLOSED (PRED-XFER-1):**
- ~~K6 got= recoverable from serial scrollback~~ → CLOSED: session row 8, got=493.0 mm (F500 K6, err −7.0 in window)

**Open (unchanged by PRED-XFER-1):**
- Hand-push encoder test (decisive motor-vs-encoder discriminator) — pending; now also
  the *formal confirmation* of the apparent post-reseat fix (rows 7–8 show R/L 1.15/1.03)
- "10s drive" duration confirmation — pending (rows 4–6 remain FAULT-flagged)
- T3: K6 F500 with telemetry (controller-alive check: pwmL/pwmR split vs 130/130) —
  pending; decides anchor hypotheses (a)/(b) of the PRED section
- K0/K3 sweep + tape metrology — postponed (pack discharged, row 9; three start-stall
  aborts at PWM_MIN 85); PRED section 5.4a holds the model predictions meanwhile
- Post-fix encoder re-verification burst (expect R/L ~1.0) — partially satisfied by
  rows 7–8 (1.15/1.03), formal pair still outstanding
- All tape_x/tape_y and volt logging — postponed
- v5 start-kick patch (200 ms @ 140, row 9 evidence: breakaway above PWM_MIN under sag)
  — pending sketch change (NOT part of DATA-XFER-1/PRED-XFER-1)

## Not touched
- the .ino sketch, controller/ISR/parser code: **no changes**
- stage3_tables.py check logic: **no changes** (new rows flow through the existing
  checks; run 7 correctly lands OUTSIDE the [-10,-6] window at err −3.7 — closer than
  one tick past the stop edge, consistent with rem 3.7 on the tick grid)
- all pre-existing REAL/PRED/TODO markers: **preserved**

## Task ID: STAGE4-AUDIT-1 — read-only fact-check of the Stage 4 outstanding-work line (17.10)

Claim audited: "1440 deg x3/dir, accuracy tables, pivot, 45 deg confirmation, 12 report
blanks — all await the next session". READ-ONLY: no file modified. Evidence per check:

1. **1440 ×3/dir — CORRECTED (split verdict).** n=3/dir IS specified with acceptance
   ranges (REPORT_STAGE4_SIM_PREFILL.md §G row 2: "1440° ×3 L + ×3 R, cruise | E +4…+8
   both dirs, same sign, σ ≤ 1.5, L/R means within 2°" — the σ≤1.5 clause is the
   dispersion rationale; the same-sign clause is the CW/CCW asymmetry check; the sketch
   comment carries "expected E(1440deg) = +4..+8 after entry. Confirm with 1440deg x3/dir").
   BUT the post-fix collapse arithmetic (1440 × 0.059 = 85° removed by the scale fix →
   expect ~+5° = run-on only) is stated only as the +4…+8 range — the explicit
   "+90 → ~+5" derivation is not written anywhere. MEASURE_STAGE4.md sec.1 still plans a
   SINGLE L1440/R1440 pair (no ×3, no dispersion rationale) and uses the stale b=160 base
   ("160.0 × f").
2. **Accuracy tables — blank count WRONG.** True count: **25 empty table cells + 20
   inline ______ blanks = 45**, not 12. (The 12 figure was a stale count of the literal
   "______" matches in an earlier grep, under-counting table cells.) Full enumeration
   below in the audit output.
3. **Pivot — CORRECTED (split).** TRACK_PIVOT=145 IS in the sketch (lines 34, 611
   greeting) and in SIM_PREFILL (§A, §G rows 3/6: pivot 360° L+R @145, pivot 15/30 ×5 ×2
   dirs, skid note in D.3). NOT in REPORT_STAGE4.md (no 145/TRACK_PIVOT/skid mention) —
   the plain report skeleton predates the pivot-track decision.
4. **45° tangent method — MISSING from all Stage 4 files.** No tangent cross-bar method,
   no L=1m checkpoints (15°→268 mm, 30°→577 mm, 45°→1000 mm), no sensitivities
   (18.7/23.3/34.9 mm/deg — arithmetic verified correct), no OUT-OF-SAMPLE flag for 45°
   (absent from the calibration fit {90°, 1440°}). §G row 5 has only "45° combined ≤
   2.25" (5% rule). The method must be added to MEASURE_STAGE4 before the session.
5. **Blank count: 45 true (25 table + 20 inline), not 12.** See enumeration.
6. **Constants consistency — CONFIRMED clean.** trackMm=151.0f / trackMmPivot=145.0f
   (sketch lines 33-34), 151/145/1.059/+4.7/3.88/±1.94 all identical in SIM_PREFILL §A/§B
   and the calibration record; worklog.md contains no 144 reference; sketch has no stale
   144 (line 66 ANG_MAX_DEG=1440 is the angle limit, not a track). grep "144\b" over the
   sketch hits only ANG_MAX_DEG.
7. **got= self-consistency — CONFIRMED.** Single source: getTrack(bool pivot) at
   lab1_stage4_turns.ino line 167; wheelTargetMM (lines 182-183), progressDeg/
   turnAchievedDeg (lines 191-193), oneCntDeg (310), profile speed (385/390/399) all
   route through it; J<mm> setter writes trackMm/trackMmPivot (485/488) with the mode
   labelled in the confirm line. No duplicated track literal in any angle path.
8. **Pre-flight preconditions — MISSING (as the audit anticipated):** neither
   MEASURE_STAGE4 nor SIM_PREFILL requires the encoder-health hand-push (R/L ≈ 1.0)
   as a gate before Stage 4 runs, nor volt logging despite session row 9 (3x start-stall
   at PWM_MIN=85 on a sagged pack). Stage 4 accuracy is void if the left encoder
   undercounts (rows 1-6 incident).

**Corrected outstanding-work line:** "pre-flight: encoder hand-push R/L≈1.0 + charged
pack w/ volt log; then 1440° ×3/dir (E +4…+8, σ≤1.5, L/R within 2°), accuracy tables
15/30/45/90 × L/R × 5, pivot 15/30 + 360° @145, 45° tangent cross-bar (needs adding to
MEASURE_STAGE4: 268/577/1000 mm @ L=1 m, OUT-OF-SAMPLE flag), 45 blank cells (25 table +
20 inline) — all await the next session"

**Status: READ-ONLY audit — no project file modified by this task.**
