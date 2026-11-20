# STATE — lab2_stage6_track (2026-11-20, consolidation)

One-page status. Diff vs `_github_upload` baseline:
`home_prep/consolidated_diff_2026-11-20.patch` (includes 19-11 + 20-11 changes).

## VERIFIED WORKING (do not change)

1. **Position detection** — `readError()` raw-threshold path:
   `trs.AnalogRead()` + `w = max(0, lineThresh - raw[i])`,
   `linePos = Σw·(i·1000)/Σw`, `error = linePos - 2000`,
   lost sentinel `-1`/`+4001` (sign = last-seen side), `LINE_LOST_MIN_WEIGHT=30`.
2. **Bench-verified at lineThresh=650** (current robot, real raw values):
   - black centred: raw 963,984,423,952,967 -> linePos=2000 error=0 lost=0
   - black under sensor 0 -> linePos=0 lost=0
   - black under sensor 4 -> linePos≈3908-4000 lost=0
   - grey only (min raw 664-700) -> lost=1
   - white only -> lost=1
3. **lineThresh survives reset** (EEPROM, confirmed after `E`).
4. **Crossing detector** — raw-based: `raw[i] < lineThresh`, >=3 sensors = crossing.

## ROBOT HISTORY

- Robot 1, 2: dead sensor front-end (all-zero RAW) — discarded.
- Robot 3: broken calibration — discarded.
- Current robot: sensors OK. Motor behaviour UNRESOLVED (below).

## OPEN PROBLEM — "rotates instead of driving straight"

Bench facts: kp=0 ki=0 kd=0, base=60, search disabled -> robot rotates on `g`;
`M` self-test printed all steps but wheels did not move at all.

EXPLAINED (2026-11-20 consolidation, fix applied):
- `M` used TEST_PWM=60 — BELOW the motor dead band (~75; PWM_MIN=85).
  The driver got PWM but the wheels cannot start that low. M was never
  dead; it was testing at an PWM the motors cannot use. Fixed: TEST_PWM=100.
- `g` rotation at base=60 CANNOT come from the drive path:
  `applyCorrection()` holds motors when `baseSpeed < PWM_MIN(85)`
  (`GUARD baseSpeed < pwmMin` — enable `t` to see it). So whatever moved
  on `g` at base=60 was NOT drive corrections.
  => Reproduce the rotation with base=90 (above PWM_MIN) before blaming signs.
- M is NOT gated by dryRun (calls motorL/motorR directly). Boot `dryRun=1`
  is irrelevant to M. M is only refused while actually driving (`s` first).
- driveWheels(): `motorL(leftSpeed); motorR(rightSpeed);` — NO swap present
  (the swap was report-only, never applied).
- SIGN_L/SIGN_R = -1/-1 are from the OLD robot (14.10) and are the prime
  remaining suspect for THIS robot once M is re-run at PWM 100.

## EXACT BENCH PROCEDURE (next session)

1. Upload latest build. Serial 9600.
2. `C` -> note `sign_l sign_r` (boot default -1 -1) and `lineThresh=650`.
3. Wheels LIFTED. Send `M`. Each step now prints its PWM (ENA=100 etc).
   - LEFT fwd: left wheel forward? RIGHT fwd: right wheel forward?
   - BOTH fwd: body-forward or spin? BOTH back: both backward?
4. One wheel backward -> `j` (left) or `J` (right), then `M` again.
5. **No motion AT ALL at PWM 100** -> power problem: battery voltage,
   H-bridge jumpers, motor supply switch. Do NOT flip signs for that.
6. Signs correct -> `E` (saves signs + gains + lineThresh + calib).
   Reset, `C` again to confirm they survived.
7. Floor test, straight section: `k0` `i0` `d0` `b90` (base MUST be >=85,
   or applyCorrection holds the motors — this is the GUARD), `g`, watch,
   `s`. If it still ROTATES with kp=0 base=90 after signs passed M ->
   hardware (H-bridge input short / one channel stuck), not software.
8. Only then steering check: lift wheels, hold line under sensor 0 -> `p`
   shows side=L error<0, and in dry-run the LEFT wheel speeds up in the
   `DRY u= L= R=` print (trace on). Floor-verify turning TOWARD the line.

## SERIAL COMMAND LIST (current sketch)

Sensor/position:
- `r` = raw ADC values
- `c` = calibrated values
- `p` = position readout (raw + lineThresh + linePos + error + lost + side)
- `t650` = set lineThresh live | bare `t` = toggle trace
- `Z` = TLC1543 debug toggle (all-6 dump + D11 idle on `r`)
Gains (live, no recompile):
- `k0.25` = kp, `i0` = ki, `d0.3` = kd, `b90` = baseSpeed, `l1000` = iLimit
- bare `d` = toggle DRY-RUN (dry-run prints DRY u/L/R instead of driving)
Motion:
- `g` = start after 3 s countdown (`s` cancels during countdown)
- `s` = stop
- `M` = motor self-test (LEFT/RIGHT/BOTH fwd, BOTH back; PWM 100)
- `j` / `J` = flip SIGN_L / SIGN_R
Search:
- `S` = toggle search enabled, `Q` = state readout, `F` = force one search
- `a120` = spin speed, `T3000` = search timeout, `n3` = found-needed
Crossing:
- `x` = toggle crossing handling
Trace:
- `R` = toggle record into buffer, `D` = dump buffer to serial
Other:
- `m` = toggle error smoothing, `z` = pidReset, `C` = full config print,
  `h` = help, `W` = 10 s warmup calibration (robot held), `w` = INERT
  (white_line no longer affects anything; kept for EEPROM compatibility)
EEPROM:
- `E` = save everything (calib, lineThresh, signs, kp/ki/kd/base)
- `L` = load, `X` = clear

## EEPROM LAYOUT v3 (magic 0xA6)

| Addr | Size | Field |
|---|---|---|
| 0 | 1 | magic 0xA6 |
| 1 | 1 | white_line (inert) |
| 2-11 | 10 | calibratedMin[0..4] |
| 12-21 | 10 | calibratedMax[0..4] |
| 22-23 | 2 | lineThresh (int LE) |
| 24 | 1 | SIGN_L (0xFF=-1, 0x01=+1) |
| 25 | 1 | SIGN_R |
| 26-29 | 4 | kp (float) |
| 30-33 | 4 | ki (float) |
| 34-37 | 4 | kd (float) |
| 38-39 | 2 | baseSpeed (int) |
| 40 | 1 | CRC8 over bytes 1..39 |

Old saves (magic 0xA5, v1/v2) rejected at load; range guards reject any
corrupt field (signs must be 1/0xFF, lt 0..1023, kp<=20, ki<=5, kd<=50,
base<=255).

## BUILD

arduino-cli 1.6.0, arduino:avr:uno, --warnings more:
0 errors, 0 sketch warnings. Flash 20216 B (62%), RAM 1523 B (74%).
