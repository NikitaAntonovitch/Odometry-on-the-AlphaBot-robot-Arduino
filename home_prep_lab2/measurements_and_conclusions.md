# LAB 2 — Stage 6 Line Follower: Measurements and Conclusions
Date of bench record: 2026-11-21. Robot: AlphaBot, Arduino UNO, TRSensors 5-channel strip.
Evidence classes used below: [HOST] = host (PC) test run, [SER] = serial-link output, [BENCH] = bench observation of the physical robot, [BUILD] = compiler/build output, [CODE] = value as written in the sketch. No timing or distance instrumentation was used in any session, so no lap times or distances are reported anywhere in this section. No completed track run is claimed.

## 1. Measurements

| # | Parameter | Value | How established | Status |
|---|-----------|-------|-----------------|--------|
| 1 | Sensor input path | AnalogRead()/readCalibrated() only; readLine() not used; position = own weighted average; lost line = sum test | [HOST] position tests 9–15 (centred / far left / far right / lost / inverted polarity / between sensors / realistic background bias), 15/15 pass | confirmed |
| 2 | lineDarkFlag (polarity) | 1 (line reads LOW calibrated values → invert 1000−v) | [SER] POLARITY line after calibration (dark-end avg vs bright-end avg) | confirmed |
| 3 | LINE_LOST_SUM_MIN | 250 | [CODE] / [SER] config print | placeholder (not tuned) |
| 4 | ON_LINE_CAL_MIN (crossing) | 300 | [CODE] | placeholder (not tuned) |
| 5 | kp=0, ki=0, kd=0, base=90 | drives straight forward | [BENCH] | confirmed |
| 6 | kp=0.05, ki=0, kd=0, base=90 | follows the line on straight and gentle curve | [BENCH] | confirmed |
| 7 | kp=0.08, base=90 (pre-clamp build) | spun (pivot) | [BENCH] | confirmed for the pre-clamp build; post-clamp retest pending |
| 8 | base=70, kp=0.05 (pre-clamp build) | spun (pivot) | [BENCH] | confirmed for the pre-clamp build; post-clamp retest pending |
| 9 | Spin root cause | D term: old kd=0.3 per tick = 15 PWM per count of error change at dt=20 ms; calibration residuals of 20–50 counts → hundreds of PWM → output saturation → inner wheel reversed | [BENCH] symptom + calculation + [HOST] D-noise test (new math bounds a 50-count residual alternation to |u| ≤ 40) | confirmed as analysis; effect of the fix retest pending |
| 10 | PID_DT_SCALE | 50 (D expressed per second) | [CODE], [HOST] D-step test | confirmed (applied) |
| 11 | kd default | 0.05 PWM per (count/s) | [CODE] | placeholder (not tuned) |
| 12 | D filter alpha | 0.25 (τ ≈ 60 ms), on the measurement, seeded after reset | [CODE], [HOST] tests 4–5 | placeholder (alpha not tuned) |
| 13 | Negation in applyCorrection() | single point, u = −uIn | [CODE] + [BENCH] (item 6: positive kp steers toward the line) | confirmed |
| 14 | Dead-band compensation | requests 1..255 → 85..255 (PWM_MIN=85); 0 = true stop | [CODE]; drive path verified by items 5–6 | confirmed (in the verified drive path) |
| 15 | Saturation clamp | |u| ≤ ±baseSpeed before L/R split | [CODE], [HOST] tests 16–19 | applied; bench regression retest pending |
| 16 | Anti-windup | conditional integration, rail ±255, iLimit=1000; pidReset clears the integrator | [CODE], [HOST] tests 20–22 (rail recovery, freeze case, reset) | applied; bench retest pending |
| 17 | Host test suite | 22/22 pass | [HOST] | confirmed |
| 18 | Build size | flash 21396 B (66 %), RAM 1525 B (74 %), 0 sketch warnings | [BUILD] arduino-cli 1.6.0, --warnings more | confirmed |
| 19 | 'Q' gate output | prints driving= and dryRun= | [SER] | confirmed (output present); gate diagnosis of item 20 pending |
| 20 | DRIVING, kp=0 ki=0 kd=0, base=90, post-anti-windup build | no motion; cause not established | [BENCH] | unexplained — pending |
| 21 | Pack voltage under load | not measured | — | open |
| 22 | ki comparison on the big curve (0.001 / 0.003 / 0.005) | not measured | — | open |
| 23 | Track run, both directions; reduced-voltage trial | not done | — | open |
| 24 | EEPROM v4 | magic 0xA7; 'E' saves gains and parameters | [SER] ('E' save / reload) | confirmed |
| 25 | trace L/R reconstruction in dumpBuffer() | base ± u WITHOUT the negation (pre-negation values) | [CODE] | caveat — do not compare trace wheel values against actual PWMs |

## 2. Conclusions

**Sensor layer.** The sensor layer follows the library's intended use: AnalogRead() for raw diagnostics and readCalibrated() for the 0..1000 per-sensor values; readLine() is not used. Position is one number — the weighted average over all five polarity-corrected values — and the lost line is a state (corrected-value sum below threshold), never a fake position. Host tests prove the arithmetic: error 0 with the line centred, about ±2000 at the strip ends, visible=false with no position when nothing is on the line, and identical geometry under inverted polarity. Polarity is established by measurement at calibration (lineDarkFlag=1 on this track), not hard-coded. On hardware, item 6 closes the loop end-to-end: the sensor path produces errors of the correct sign, because the robot steers toward the line.

**Motor layer.** Items 5 and 6 prove the motor layer and its signs: with the controller output zero the robot drives straight forward (both wheels balanced, SIGN_L/SIGN_R correct as configured, dead-band compensation active), and with kp=0.05 the steering sign is correct in the applied direction. The search spin deliberately runs on a separate path that bypasses the ±baseSpeed clamp, because a spin requires opposite wheel directions.

**Controller fixes — what each changed.**
1. *D per second + filtered measurement + seed* — removed the noise amplifier: the old per-tick D turned 20–50-count calibration residuals into hundreds of PWM and saturated the output (the observed spin). The new D is bounded by host test (|u| ≤ 40 under a 50-count residual alternation).
2. *Single negation in applyCorrection()* — made positive kp steer toward the line; bench-confirmed by item 6.
3. *Dead-band compensation* — small correction requests now reach the wheels (mapped onto 85..255) while 0 stays a true stop; the drive path is verified by items 5–6.
4. *Clamp of u to ±baseSpeed* — removed the pivot mechanism: at full correction the inner wheel is commanded to stop, never to reverse. The pre-clamp spins (items 7–8) are not expected to recur; the retest is pending.
5. *Conditional-integration anti-windup* — the integrator freezes at the output rail instead of winding to iLimit, so a sign change of the error takes effect on the next tick instead of after hundreds of ticks of unwinding; pidReset() clears the integrator. Bench retest pending.

**FLAG — residual windup band.** The anti-windup rail is ±PWM_MAX = 255, but the drive path additionally saturates the correction at ±baseSpeed (= 90) inside applyCorrection(). A correction requested anywhere in the 90..255 band is clamped by the drive path while the anti-windup still sees the output as "not at the rail", so the integrator can wind up inside that band with no visible effect on the wheels. Recommendation: use min(PWM_MAX, baseSpeed) as the anti-windup rail (listed as open item 8).

**What remains open.** The frozen-DRIVING case of item 20 is unexplained (voltage under load not measured); the clamp and anti-windup fixes have not been re-tested on the robot; kd, LINE_LOST_SUM_MIN, ON_LINE_CAL_MIN and the filter alpha are placeholders; ki has not been systematically compared; the track has not been completed in either direction; no reduced-voltage trial has been made; and the trace dump's wheel values are pre-negation (item 25) and must not be compared against actual PWMs until dumpBuffer() is fixed.

## 3. PID justification (for the demo)

**P — kp = 0.05 (at base = 90), hardware-confirmed.** Evidence: with kp = 0 the robot drives straight forward, so P is the only steering term; with kp = 0.05 it follows straights and the gentle curve (bench, item 6). At too high a gain the robot over-corrects: kp = 0.08 on the pre-clamp build spun — although that build also carried the unfixed D term, so that observation bounds the old design rather than kp alone (post-clamp retest pending; the classic too-high symptom, zigzag about the line, has not yet been measured on the current build). At too low a gain the steering authority is insufficient: the big curve currently loses the line slightly at kp = 0.05, though the split between kp, ki and speed for that loss is not yet established.

**I — ki = 0 in the confirmed configuration.** The line follower crosses zero error continuously, so integral action is used cautiously. ki = 0.001 was loaded once and produced the frozen-DRIVING failure: the integrator wound up against the output clamp and then unwound at only ~1 PWM per tick, keeping the output pinned (bench-observed; root-caused and fixed by conditional integration, host tests 20–22; bench retest pending). At too high a gain: windup against the rail — observed — with the additional caveat that the current rail (±255) sits above the drive-path saturation (±baseSpeed), leaving a 90..255 windup band; the recommended rail is min(PWM_MAX, baseSpeed). At zero (or too low): no compensation of a steady offset on long constant-radius curves; the slight big-curve loss is consistent with this, but attribution waits for the planned ki comparison (0.001 / 0.003 / 0.005 on the same section), which has not been measured.

**D — kd = 0.05 PWM per (error count per second), placeholder value.** The evidence for the unit change is the bench failure it fixed: the old kd = 0.3 acted per tick — 15 PWM per count of error change at dt = 20 ms — so 20–50-count calibration residuals produced hundreds of PWM, saturated the output and reversed the inner wheel (the observed spin). The new D acts on a first-order filtered measurement (alpha 0.25, τ ≈ 60 ms, seeded after reset) and is host-bounded: the same residual pattern now yields |u| ≤ 40. At too high a gain D amplifies sensor noise and saturates the output — observed with the old math. At zero or too low there is no damping of zigzag; that symptom has not yet been measured on the current build.

## 4. Line-search strategy justification

When the sum-based lost line fires (no sensor on the line), the robot spins in place toward the side of the LAST VALID error — the error value is carried over from the last visible reading, so the search direction is the side where the line was last seen. A spin was chosen over an arc or reverse motion because it does not carry the robot away from the point where the line was lost; its known limitation is that it cannot find a line lost "ahead" (sharp bend taken on the outside), and the timeout stop is the fallback there. Returning requires foundReadingsNeeded = 3 consecutive good readings about one control period apart, because a spinning strip can sweep across the line accidentally in a single reading; after searchTimeoutMs (default 3000 ms) the robot stops and waits, which is the safe behaviour at a genuine end of line. The parameters (spin speed 120, timeout 3000 ms, N = 3) are sketch defaults: not tuned, and the search has not yet been exercised on hardware in the current build session.

## 5. Open items — with the exact next test

1. **Frozen DRIVING at kp=0 ki=0 kd=0 base=90 (post-anti-windup build), cause not established.** Next test: from the serial link send 'Q' and record driving= and dryRun=; then measure the pack voltage under load (never measured); if the voltage is sound, run 'M' with the wheels lifted to exercise the dead-band-compensated motor path at PWM 100; only then repeat 'g'.
2. **Clamp regression (items 7–8).** Next test: re-run kp=0.08 at base=90 and kp=0.05 at base=70 on the current build; expected: arc with the inner wheel stopped, no pivot. Record both runs.
3. **Anti-windup freeze case.** Next test: re-run kp=0.05, ki=0.001 (the previously frozen combination) and confirm it drives; capture 'Q' before and after.
4. **Anti-windup rail band (FLAG).** Next step: change the rail in pidStep() to min(PWM_MAX, baseSpeed), re-run the host suite, then repeat open item 3.
5. **ki comparison.** Next test: same big-curve section, same base speed, ki = 0.001 / 0.003 / 0.005 in sequence; record completion of the section per value. Not measured so far.
6. **kd tuning.** Next test: 'd<val>' sweep around 0.05 with trace recording on; stop when zigzag is damped without noise-driven chatter.
7. **LINE_LOST_SUM_MIN tuning.** Next test: 'p' over the line vs over background, record both sums, set the threshold between them, save with 'E'.
8. **ON_LINE_CAL_MIN tuning.** Next test: watch the corrected values ('p') while driving over the crossing stripe; adjust 300 if the crossing is not detected or false-positives appear.
9. **Track completion + reduced-voltage trial.** Next test: full track in both directions at the confirmed settings; then repeat at reduced pack voltage and record the behaviour change. Not started.
10. **dumpBuffer() negation caveat.** Next step: change the L/R reconstruction to base−u / base+u (negation included) before using any trace dump for wheel-PWM comparisons.
11. **Filter alpha placeholder.** Next test: only if D chatter reappears in recorded traces, sweep alpha below 0.25; otherwise leave.
