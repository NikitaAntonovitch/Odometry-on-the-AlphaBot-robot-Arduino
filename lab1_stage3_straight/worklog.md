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
