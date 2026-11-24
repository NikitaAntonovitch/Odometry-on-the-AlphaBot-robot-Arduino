# lab2_stage6_trsensors — STATE (2026-11-22)

TRSensors-library refactor of lab2_stage6_track (21.11 teacher-fixed version).
All sensor handling now goes through the library; PID/motors/state machine/
serial commands unchanged.

## Sensor layer (what replaced what)
| old (stage6_track)                        | new (this sketch)                    |
|-------------------------------------------|--------------------------------------|
| computePosition() + 1000-v inversion loop | trs.readLine(sensorValues, whiteLine)|
| lineDarkFlag / measurePolarity...()       | whiteLine constant (bench 09-11: 0)  |
| sum < lineLostSumMin (250) lost test      | library: no stored value > 300 = lost|
| readError() keeps lastGoodError           | readLine -1 / 4001 sentinels, e=+-2001|
| trs.calibrate() x400                      | unchanged (already library)          |
| trs.AnalogRead ('r'), readCalibrated ('c')| unchanged (already library)          |

## Key facts
- readLine() calls readCalibrated() internally (1 ADC read per tick), writes
  the corrected values (1000-v for our dark line) back into sensorValues[] -
  crossing test + 'p' print reuse that array, no re-read.
- error is SIGNED int; lineLost(e) = abs(e) > 2000. The library example's
  unsigned int is WRONG for us (-1 would wrap to 65535).
- last_seen side is remembered INSIDE readLine (static last_value, initial
  LEFT); our lastSeenSide derives from the sign of the +-2001 error.
- EEPROM layout v5 (magic 0xA8): whiteLine replaces lineDarkFlag,
  lineLostSumMin dropped (library-owned now). Old v4 saves rejected by magic.
- 'w' flips whiteLine (bench override), 'E' saves, boot/W print MIN/MAX arrays
  as polarity sanity (MIN range = dark end = the line on this track).
- t<val> (lost-sum threshold) no longer functional: prints an explanation.
  Bare 't' still toggles trace.

## Verification (2026-11-22)
- Compile: arduino-cli, arduino:avr:uno, --warnings more, --library
  ../../TRSensors/TRSensors: 0 errors, 0 warnings from the sketch (only
  pre-existing TRSensors.cpp warnings).
  Flash 20854 B (64%) / RAM 1523 B (74%), 525 B free.
  Old sketch for reference: 21396 B (66%) / 1525 B.
- Differential host test home_prep/differential_lib_test.cpp: 19/19 PASS,
  2 EXPECTED-DIFF (library thresholds replace ours):
    A) stored sum 250..max<=300: old visible, new LOST (library stricter)
    B) background residual <=50: library excludes it from the average,
       old included it (e shifts by ~tens)
  All envelope cases identical: centred/edges/straddles (exact), lost + side
  memory, crossings (arrays byte-identical, isCrossing same), recovery,
  white_line=1 mirror of lineDarkFlag=false, PID u-sequence identical.
- NO HARDWARE verification yet. First bring-up: hardware_checklist_2026-11-21
  still applies, EXCEPT: no POLARITY print after calibration (MIN/MAX arrays
  + 'p' instead), EEPROM = v5 (old saves rejected -> expect 10 s calibration
  on first flash), and the lost-line threshold is fixed (no t<val> retune).

## Defaults (unchanged from 21.11 session)
kp=0.25 ki=0.0 kd=0.05 base=90 search=1 smoothing(LPF alpha 0.25)=always-on
spin=120 timeout=3000 foundNeeded=3 crossing=on(threshold 3, hold 200 ms)
whiteLine=0 (dark line, bench-measured 09-11)
