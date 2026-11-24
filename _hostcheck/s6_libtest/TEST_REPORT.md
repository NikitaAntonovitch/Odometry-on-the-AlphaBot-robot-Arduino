# LAB 2 (lab2_stage6_trsensors) — no-hardware test report
2026-11-22. Tested by Hermes on the host PC. Sketch files NOT modified — all test
code lives in `_hostcheck/s6_libtest/`. Tools: arduino-cli 1.6.0 (Arduino IDE bundle),
g++ 16.1 (MSYS2 ucrt64), the real TRSensors library source, both sketch sources,
both assignment PDFs.

## What was tested and how

Three independent methods, in increasing scope:

1. **Code inspection** — TRSensors.cpp/.h, lab2_stage6_trsensors.ino (1158 lines),
   lab2_stage6_track.ino (1269 lines), assignment PDF + 21-page implementation guide
   read in full.
2. **Unit tests of the REAL library** (30 checks, all PASS) — `sensor_path_test.cpp`
   compiles the project's actual `TRSensors/TRSensors/TRSensors.cpp` unmodified against
   a pin-level TLC1543 simulator (the mock Arduino core emulates the CS/ADDRESS/CLOCK/
   DATA protocol of the TLC1543, so the library's bit-bang code runs for real).
3. **Full-sketch simulation** (93 checks, all PASS) — `sketch_sim_test.cpp` compiles
   the entire `lab2_stage6_trsensors.ino` unmodified on the host and drives it through
   its real serial parser, checking motor PWM/H-bridge pins, LED, Serial output and
   EEPROM bytes.
4. **Differential test** — both sketches (old + new) compiled side by side, fed an
   identical 133-tick sensor scenario (centred → slide-off → lost/search-spin →
   re-acquire → crossing stripe → noisy background), wheel/LED outputs diffed
   byte-for-byte, plus targeted probes of the internal error/PID differences.
5. **Static audit** — signed/unsigned, overflow, uninit, bounds, RAM.

Reproduce any of it:
```
cd C:\4work\study\ISS\LAB\LAB2_bring_to_TRSensors
# unit tests (real library):
g++ -std=c++17 -I_hostcheck/s6_libtest -ITRSensors/TRSensors \
  -include _hostcheck/s6_libtest/Arduino.h -x c++ \
  TRSensors/TRSensors/TRSensors.cpp _hostcheck/s6_libtest/sensor_path_test.cpp \
  -o _hostcheck/s6_libtest/sensor_path_test.exe && _hostcheck/s6_libtest/sensor_path_test.exe
# full sketch sim:
g++ -std=c++17 -I_hostcheck/s6_libtest -ITRSensors/TRSensors \
  -include _hostcheck/s6_libtest/Arduino.h -x c++ \
  TRSensors/TRSensors/TRSensors.cpp lab2_stage6_trsensors/lab2_stage6_trsensors.ino \
  _hostcheck/s6_libtest/sketch_sim_test.cpp -o _hostcheck/s6_libtest/sketch_sim_test.exe \
  && _hostcheck/s6_libtest/sketch_sim_test.exe
```

## Results by area

| Area | Result | Evidence (short) |
|---|---|---|
| 1. Compile Uno + warnings | PASS | see below; 0 sketch warnings both sketches |
| 2. Library use: calibration / readings / position / polarity / lost | PASS | 30 unit checks on real library code; sketch calls verified against guide |
| 3a. Position: centred/edges/between | PASS | p=0→pos 0, p=2→2000, p=4→4000, p=1.5→1500, monotonic |
| 3b. Lost both sides + recovery | PASS | −1 (left memory), +4001 (right), 3-tick re-acquire |
| 3c. Noisy readings | PASS | >50 gate exact (50 excluded, 51 in); residual-17 background OK |
| 3d. Crossings | PASS | stripe detected, 200 ms hold verified at pin level |
| 3e. Both polarities | PASS | white_line=0/1 mirror OK; wrong arg → mirrored position (documented) |
| 4. PID / limits / dead band / search / timeouts / timing / rollover | PASS | 93-check sim incl. millis-wrap and catch-up tests |
| 5. Serial cmds + invalid input + EEPROM v5 | PASS | all commands; v4 0xA7 rejected; CRC + range guards verified |
| 6. Old-vs-new differential | PASS with 1 behavioral diff | see "Difference #1" |
| 7. Uno-specific audit | PASS with 2 notes | signed-int discipline correct; see "Notes" |

### 1. Compile (arduino-cli 1.6.0, arduino:avr:uno, --warnings more)

```
lab2_stage6_trsensors:  Sketch uses 20854 bytes (64%) program, 1523 B (74%) RAM, 525 B free.
lab2_stage6_track:      Sketch uses 21396 bytes (66%) program, 1525 B (74%) RAM, 523 B free.
```
- Sketch code: 0 errors, 0 warnings in BOTH sketches.
- Library file warnings (pre-existing, harmless, also in the 21.11 build): 8×
  `-Wchar-subscripts` (char array index — safe here: j/q ∈ 0..5) + 3 unused vars,
  all in TRSensors.cpp, none in either .ino.

### 2. Library usage vs the assignment guide
- `readLine(sensorValues, whiteLine)` with a SIGNED int error — exactly what the
  guide's stage 2 mandates (the unsigned trap is real: verified −2001 stored in a
  16-bit unsigned becomes 63535; the sketch is immune).
- `error = readLine() − 2000`; `lineLost = |e| > 2000` — matches the guide's table
  (−1/4001 → ±2001). Verified end-to-end: line at sensor0 → `error=-2000 side=L`,
  sensor4 → `error=2000 side=R`, off-line right → `error=2001 lost=1 side=R`.
- Calibration = 400× `trs.calibrate()` (guide's pattern); boot loads EEPROM v5 first,
  calibrates only on miss; MIN/MAX printed; 'W' warmup re-calibrates.
- Polarity: `whiteLine` is NOT auto-measured in the new sketch (the old sketch
  measured it). It is a compiled-in default 0 + bench override 'w' + EEPROM save.
  This is a declared design change — see Difference #3.
- readLine internally: `readCalibrated` → invert `1000−v` when white_line=0 →
  on_line if `value > 300` (strict) → weighted average over `value > 50` (strict).
  All three verified exactly (300 → lost, 301-equivalent → visible; 50 excluded,
  51 included). Doc-vs-code bug in the library header confirmed: with white_line=0
  it INVERTS values while claiming dark line = no invert — irrelevant here because
  white_line=0 is correct for our dark line.

### 4. Control path (full-sketch sim, pins checked)
- PID: step e:0→−1000 at kp=0.25 → |u| saturates → applied clamp → L=180/R=0
  request; dead-band map verified (90→145, 120→165, 180→205; 0→true stop).
- ±baseSpeed clamp: inner wheel never reverses (R=0 at full correction, both sides).
- base<PWM_MIN guard: b10 → motors held at 0 PWM even while driving.
- Search: lost → LED on, spin 165/165 with correct H-bridge directions; 3
  consecutive good readings required ('n1' shortens it); timeout 3000 ms → stop +
  message, stays stopped, recovers when line reappears; Bug-1 fix verified (leaves
  SEARCHING even with search disabled).
- Crossing: stripe (sensors 0/2/4 on line) → hold last correction 200 ms (verified
  PWM frozen at held value), then PID resumes and settles; 'x' disables cleanly.
- Loop timing: 150 ticks over 3 s wall → exactly 150 records (no drift); after an
  idle gap each loop() runs at most ONE catch-up tick (no burst); dt window sane.
- millis() rollover: clock parked at 2^32−6 — search timeout fires correctly across
  the wrap (unsigned arithmetic), recording continues past the wrap, dt window sane.

### 5. Serial + EEPROM
- Every documented command exercised through the real parser. Invalid input:
  garbage numbers → 0, unknown letters → `? x`, 60-char line → truncated safely
  (23 chars), empty line → ignored, no crash anywhere.
- NOTE found by testing: `k-3` (negative kp) is accepted without any range guard
  (EEPROM load DOES guard kp to 0..20, but the serial command does not). Not a
  crash risk; behavior would be "steers away" — flagging as a minor gap only.
- EEPROM v5: E→RAM-change→L round-trip restores kp/base (byte layout verified at
  addresses 24..27/36..37, CRC at 38); v4 magic 0xA7 rejected; corrupted CRC →
  "no saved calibration"; kp=1000 / sign=0x02 / whiteLine=7 all rejected by the
  per-field guards; 'X' clears.

### 6. Differential old-vs-new (the part you most need to read)

Identical 133-tick scenario through both sketches (native EEPROM layout primed for
each, same gains kp=0.25/ki=0/kd=0.05/base=90, dark line, signs +1/+1):

- **131 of 133 ticks byte-identical** in wheel PWM + H-bridge direction pins + LED.
- **Difference #1 (real, boundary, 1 tick):** when the line slides off the LEFT
  edge, the new sketch loses the line one tick EARLIER. Cause (verified in the
  unit tests): at that geometry the edge sensor's inverted value is exactly 300 —
  the library's on-line test is strict `>300` (→ lost), while the old sketch's sum
  test (`sum=300 ≥ lineLostSumMin=250`) still calls it visible. One 20 ms tick of
  earlier search entry; spin direction and everything after is identical.
  This is an inherent, declared library-threshold difference, not a bug.
- **Difference #2 (measured, small, hidden by the wheel clamp):** over a realistic
  noisy background (calibrated residual ~17), the old path averages ALL corrected
  values (residuals included, e.g. e=−185 at line-position 1.8), the new path
  excludes residuals ≤50 from the average (e=−211 there — the error is LARGER by
  ~26 counts because the noise no longer dilutes the average). At kp=0.0005 the
  converged raw u differs by exactly this e-difference (u=0 vs u=−1); at real
  gains the ±baseSpeed wheel clamp makes the outputs identical, which is why the
  pin-level diff showed 0 difference in that phase. Impact at demo gains: none on
  the pins; the internal error is a few tens of counts larger on the new sketch.
- **Difference #3 (declared design change):** polarity handling. Old: measured at
  every calibration (dark-end vs bright-end averages) + EEPROM flag. New: fixed
  whiteLine=0 default + 'w' manual override + EEPROM save; NO auto-measurement.
  The new boot does print MIN/MAX so you can sanity-check the dark end by eye.
  Acceptable under the assignment (polarity must be known from measurement — it
  was measured 09-11 and is baked in), but there is no runtime re-measure if the
  track/polarity changes: you must use 'w' + 'E'.
- Difference #4 (declared): lost-line threshold no longer tunable ('t<val>' now
  prints an explanation; bare 't' still toggles trace). The library's >300 test
  replaces the tunable sum threshold.

## Notes / minor findings (ordered by importance)

1. **Calibrate() crossed min/max update** (library, verified): `calibratedMax` is
   updated from the call's MIN and `calibratedMin` from the call's MAX, so extremes
   only ever widen. Harmless in practice (ranges converge to the true extremes),
   but it means a one-off spurious value can never be removed by later reads —
   recalibration is the only reset. No action needed.
2. **readLine's side memory is a function-static**, not a member: survives even
   replacing the TRSensors object; boot default = LEFT (−1). The sketch handles
   this correctly (search direction from the sentinel sign). No action needed.
3. **Uncalibrated readLine can never report "lost"**: with constructor-default
   ranges (min=1023/max=0) every sensor calibrates to ~0 → inverts to ~1000 → all
   "on line" → returns fake 2000. In this sketch impossible (setup always
   calibrates or loads EEPROM before loop). No action needed; do not add sensor
   reads before calibration in future edits.
4. **`k<val>` accepts negative kp** while the EEPROM loader rejects it (guard
   0..20). Suggest (with your approval) clamping in the command handler like 'a'
   does for spin speed.
5. **Trace-dump L/R semantics**: dumpBuffer prints `L=base+u, R=base−u` while the
   applied wheels are `L=base−u_applied...` — the STATE.md of the old sketch
   already carries this caveat; it is unchanged in the new sketch (raw u recorded,
   negation happens in applyCorrection). Don't compare trace L/R to real PWMs.
6. Host-vs-AVR arithmetic divergence in the library (host testing only, not the
   robot): `readCalibrated`'s out-of-range clamp behaves differently on 32-bit-long
   hosts; on AVR it is correct. Documented in the test file; irrelevant to the
   robot.

## What still requires the real robot

- **Motor signs / M self-test** (SIGN_L/R are EEPROM-loaded here; 'j'/'J' flow) —
  sim cannot verify real wheel directions.
- **Actual TLC1543 timing** (real SPI bit-bang timings vs simulator; real noise).
- **Calibration on the real track** (real MIN/MAX ranges, grey-floor reading,
  polarity confirmation via 'p' + MIN/MAX print).
- **PID tuning**: kp=0.25, kd=0.05 are placeholders; kd needs bench tuning
  (d<val> sweep); ki comparison per the guide.
- **Search/crossing in real geometry**: spin speed 120 vs real floor, crossing
  threshold 3 vs the real transverse stripe, boundary flip-flop behavior.
- **The 1-tick earlier lost-line entry** (Difference #1): confirm on the track
  that sliding off the edge still recovers (it should — one tick earlier).
- EEPROM save/load on real hardware after a full calibration ('E' → power cycle →
  boot banner should show "calibration loaded").

## Verdict

The refactored sketch compiles clean, uses the library exactly as the assignment
guide requires, and its observable robot behavior (motors, LED, state machine,
EEPROM) is **equivalent to the 21.11 teacher-fixed sketch** across a 133-tick
identical-input differential — except for the four declared/verified differences
above, of which only the polarity auto-measure removal (#3) needs a habit change
at the track ('w' + 'E' instead of automatic re-measure).
