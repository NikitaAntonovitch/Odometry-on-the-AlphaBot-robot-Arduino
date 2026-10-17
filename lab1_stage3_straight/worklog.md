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

## Open TODO inventory (unchanged, nothing closed)
- K6 done line (got=) — recover from serial scrollback if user provides it
- Hand-push encoder test (decisive motor-vs-encoder discriminator) — pending
- "10s drive" duration confirmation — pending
- T3: K6 F500 with telemetry (pwmL/pwmR split vs 130/130) — pending
- Post-fix encoder re-verification burst (expect R/L ~1.0) — pending
- All tape_x/tape_y and volt logging — postponed until fault resolved

## Not touched
- the .ino sketch, controller/ISR/parser code: **no changes**
- REPORT_STAGE3*.md existing REAL/PRED/TODO markers: **preserved**; the
  limitations paragraph was inserted as new text without altering existing
  sections
