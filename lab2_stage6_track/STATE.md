# STATE — lab2_stage6_track (2026-11-21 PM: PID D-fix + sign fix + dead-band comp on top of the teacher-fixed sensor path)

Bench 21.11: spin happens with kp=+0.25, kp=-0.25 AND kp=0 (kd=0.3) -> the D
term, not P sign, was the driver: per-tick kd=0.3 = 15 PWM per count of error
change at dt=20 ms; 20-50 count calibration residuals -> hundreds of PWM ->
saturation -> one wheel reversed. Three fixes applied (all compile-verified,
host-tested 15/15 [SIM], NOT hardware-verified yet):

1. D PER SECOND: d = kd * dfilt * (1000/LOOP_PERIOD_MS); kd default now 0.05
   PWM per (count/s), PLACEHOLDER - NEEDS HW TUNING (start 0.02-0.1). u clamped
   to +-255 inside pidStep AND per-wheel in applyCorrection.
2. D on the MEASUREMENT via first-order LPF (alpha 0.25, tau=60 ms), filter
   output vs filter output ONLY (the old smoothed-vs-raw comparison is gone).
   Filter is SEEDED on the first tick after every pidReset (filt=raw, d=0) -
   without the seed, e=1000 at start = 625 PWM cold kick. 'm' now just reports
   (smoothing always on); smoothError variable removed.
3. SIGN: correction negated ONCE in applyCorrection (u = -uIn) - positive kp
   now follows the line. SIGN_L/SIGN_R untouched (M self-test is the authority).
   Trace dump semantics: lastU/lastGoodU = RAW pidStep output; L/R printed as
   base-u / base+u (negation included). NOTE: dumpBuffer still prints
   baseSpeed+row.u / baseSpeed-row.u - update at next dump review if checked.
4. DEAD-BAND COMP: deadBandCompensate() in motorL/motorR maps requests
   1..255 -> 85..255 linearly (PWM_MIN=85), 0 = true stop. All paths (drive,
   search spin, M self-test) get it; printed L/R values are pre-compensation.
5. SATURATION CLAMP (bench 21.11 PM, VERIFIED ON HW root cause): |u| clamped
   to +-baseSpeed in applyCorrection BEFORE L/R split -> inner wheel stops at
   full correction, NEVER reverses. This is what killed the spin at kp=0.08
   and base<85 (a small negative inner request used to become -85 PWM reverse
   via dead-band comp). Search spin bypasses the clamp via driveWheels()
   directly (opposite wheel directions are REQUIRED for a spin).
6. ANTI-WINDUP, conditional integration (21.11: ki=0.001 + clamp -> DRIVING
   but frozen): pidStep checks the pre-clamp output FIRST and skips the
   integration when uPre is at a +-255 rail AND ki*error pushes further into
   that rail. Old math wound integral to iLimit=1000 and unwound at only
   1 PWM/tick -> u pinned at 255 for ~750 ticks after the error flipped.
   New math: integral freezes at ~255.5 (rail), NOT 1000; sign flip lowers u
   on the very next tick. pidReset() clears the integrator (integral=0).
   iLimit remains the hard cap; clamp/negation/signs/driveWheels untouched.

BENCH CONFIRMED 21.11 PM (current robot, hardware-verified):
- kp=0 kd=0 base=90 -> drives STRAIGHT (signs + drive path correct)
- kp=0.05 kd=0 base=90 -> FOLLOWS the line (sign convention correct)
- Spin at base=70 / kp=0.08 -> root-caused to negative inner wheel +
  dead-band amplification -> fixed by the clamp (5). Re-test pending.
- ki=0.001 + clamp -> DRIVING but no motion -> root-caused to integrator
  windup against the rail -> fixed by (6). Re-test pending.

## SENSOR PATH (teacher-fixed, from the morning session - unchanged)

- computePosition(bool &lineVisible): readCalibrated -> invert 1000-v when
  lineDarkFlag -> sum; sum < lineLostSumMin (250, NEEDS HW TUNING) -> lost,
  no fake position; else weighted average (0..4000), error = pos - 2000.
- Polarity by measurement (measurePolarityFromCalibration), EEPROM v4 (0xA7).
- readLine() and raw thresholds GONE. 'p' shows corrected cal + sum + visible.

## HOST TESTS (pid_test.cpp, 15/15)

1-3 P/I (gain line, clamp), 4 D-step per second (seed -> spike -> decay),
5 D-noise bounded (50-count residual noise -> u <= 40), 6 pidReset,
7 windup, 8 combined sine, 9-15 computePosition (centred/far-L/far-R/lost/
inverted polarity/between-sensors/realistic bias).

## ROBOT HISTORY

- Robot 1, 2: dead sensor front-end - discarded. Robot 3: broken calibration -
  discarded. Current robot: sensors OK, motor signs verified via M (after fix 3
  the spin cause moved to D saturation - see top).

## STILL OPEN (verify on hardware, in this order)

1. Re-test the clamp fix: kp=0.08 base=90 (previously spun) and base=70
   (previously spun) - both should now arc with the inner wheel stopped,
   never pivot in place.
2. Re-test the anti-windup fix: ki=0.001 with kp=0.05 (previously frozen)
   - must now drive; also verify 'Q' prints driving/dryRun on the link.
3. kd tuning live: d<val>, watch trace R/D for zigzag damping at kp=0.05.
4. lineLostSumMin / ON_LINE_CAL_MIN: 'p' sums over line vs background.
5. dt_avg still ~20 ms (LPF adds negligible time).
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

## SERIAL COMMAND LIST (current sketch, 21.11 PM)

Sensor/position:
- `r` = raw ADC values
- `c` = calibrated values
- `p` = position readout (corrected cal values + sum + error + visible + side)
- `t250` = set lineLostSumMin live | bare `t` = toggle trace
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
- `m` = reports D smoothing always on (LPF alpha 0.25), `z` = pidReset,
  `C` = full config print, `h` = help, `W` = 10 s warmup calibration
  (robot held, re-measures polarity), `w` = flip lineDarkFlag (bench override)
EEPROM:
- `E` = save everything (calib, lineThresh, signs, kp/ki/kd/base)
- `L` = load, `X` = clear

## EEPROM LAYOUT v4 (magic 0xA7)

| Addr | Size | Field |
|---|---|---|
| 0 | 1 | magic 0xA7 |
| 1 | 1 | lineDarkFlag (measured polarity) |
| 2-11 | 10 | calibratedMin[0..4] |
| 12-21 | 10 | calibratedMax[0..4] |
| 22-23 | 2 | lineLostSumMin (int LE) |
| 24 | 1 | SIGN_L (0xFF=-1, 0x01=+1) |
| 25 | 1 | SIGN_R |
| 26-29 | 4 | kp (float) |
| 30-33 | 4 | ki (float) |
| 34-37 | 4 | kd (float) |
| 38-39 | 2 | baseSpeed (int) |
| 40 | 1 | CRC8 over bytes 1..39 |

Old saves (magic 0xA5/0xA6, v1/v2/v3) rejected at load; range guards reject
any corrupt field (signs must be 1/0xFF, lostSumMin 0..1000, kp<=20, ki<=5,
kd<=50, base<=255).

## BUILD

arduino-cli 1.6.0, arduino:avr:uno, --warnings more:
0 errors, 0 sketch warnings. Flash 21178 B (65%), RAM 1525 B (74%).
