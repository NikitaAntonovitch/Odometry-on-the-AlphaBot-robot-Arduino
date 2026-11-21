# Hardware verification checklist — teacher-fixed sensor path (2026-11-21)

The computePosition() redesign (readCalibrated weighted average + sum-based
lost line + measured polarity) is compile-verified and host-tested (15/15 in
pid_test.cpp) but NOT verified on the robot. NOTHING below is claimed fixed.

## A. First bring-up (bench, wheels up)

1. Flash lab2_stage6_track. Expect boot banner; if EEPROM has an old v3 block
   (magic 0xA6) it is rejected -> it runs the 10 s hand-rotation calibration.
2. During that calibration: sweep the strip across line + background for the
   FULL 10 s (stationary calibration = flat ranges = polarity check warns).
3. Watch the new line after "CALIBRATION DONE":
   `POLARITY: dark-end avg=... bright-end avg=... -> lineDarkFlag=1 (line LOW -> invert 1000-v)`
   - On our track (dark line) it MUST print lineDarkFlag=1. If 0: the calibration
     did not see the line (or the track really is bright-line) - recheck 'p'.
4. 'c' -> CAL values: over line LOW, over background HIGH (calibrated view).
5. 'r' -> RAW still works (diagnostics only, not used for position).

## B. Position path ('p' command)

6. Line centred under sensor 3 (idx2): `error=` about 0 (small +/-60 bias is
   EXPECTED from background residuals - see host test 15).
7. Line under IR1 (idx0): error about -2000, side=L.
8. Line under IR5 (idx4): error about +2000, side=R.
9. Walk the strip slowly L->R across the line: error must go negative -> 0 ->
   positive, monotonic, no flips. THIS is the teacher's core complaint -
   verify it before anything else.
10. Lift the robot OFF the track (strip over nothing / uniform background):
    `visible=0`, and `error=` stays at the last valid value (NOT a fake
    position). Then back on the line: `visible=1` again.
11. Record the `sum=` values shown by 'p': over line (typ. several hundred to
    >1500), over background (typ. < 250). Pick lineLostSumMin between them:
    't<val>' live, then 'E' to save. Current default 250 = PLACEHOLDER.

## C. Search & crossing

12. 'F' (force one lost tick) with robot held: state -> SEARCHING, side comes
    from the last valid error sign. Spin direction correct for the wiring?
    (Motor signs themselves are STILL the open 'j/J' question - D below.)
13. Return to line: 3 consecutive good readings required - does a fast spin
    sweep past the line without 3 hits? (Known suspicion from stage 5 [HW-PENDING].)
14. Crossing: drive over the transverse stripe - LED blinks, holds last
    correction 200 ms. ON_LINE_CAL_MIN=300 and CROSSING_THRESHOLD=3 both
    NEED TUNING (watch 'p' sums while crossing).

## D. OPEN PROBLEM carried over — "rotates instead of driving straight"

UNRESOLVED from 2026-11-20 (see lab2_stage6_track/STATE.md). Bench facts:
kp=0 ki=0 kd=0, base=60 -> rotates on 'g'; at base=60 the drive path holds
motors (baseSpeed < PWM_MIN guard), so that rotation was NOT drive corrections.
Steps agreed in STATE.md, still pending:
15. Reproduce with base=90 (above PWM_MIN) before blaming signs.
16. 'M' self-test (now TEST_PWM=100, above the ~75 dead band): wheels must roll
    fwd/fwd/fwd/back. If a wheel runs backward -> 'j'/'J' -> M again -> 'E'.
17. Only AFTER 15-16 pass: kp=0 base=90 'g' on the floor = both wheels equal
    speed = straight. Any rotation now IS a sign/direction bug -> j/J/M.

## E. Regression (after B-D pass)

18. 'g' countdown start, dry-run trace with 't' + 'R'/'D' dump: e in
    +-2000, no sentinels in normal following (t=-1 only on real loss now).
19. dt_avg still 20 ms / over=0 (readCalibrated adds ~2-3 ms per tick vs the
    raw path - watch overCount; if late ticks appear, lower nothing yet,
    just record it).
20. 'E' save + power cycle: calibration + polarity flag + lineLostSumMin
    survive (EEPROM v4, magic 0xA7).
