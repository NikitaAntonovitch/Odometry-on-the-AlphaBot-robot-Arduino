# Session notes — 2026-11-09 — LAB2 / lab2_stage6_track.ino

## 1. Final state of lab2_stage6_track.ino

- Defaults baked in: kp=0.25, ki=0.0, kd=0.3, baseSpeed=90, searchEnabled=true,
  smoothError=true — **[HARDWARE-VERIFIED in code, boot print [HARDWARE-PENDING]]** (compile-verified; see known issue #1)
- `g` = 3 s countdown start (y-confirmation removed); `s` during countdown →
  CANCELLED; on expiry: state machine reset (state=FOLLOWING, searchStartTime=millis(),
  searchFoundCount=0, searchTimedOut=false, crossingUntilMs=0, LED off) then
  driving=true, dryRun=false, pidReset(), DRIVING line — **[code-verified, hardware-pending]**
- `setup()` banner now prints real defaults ("defaults: kp=0.250 kd=0.300 base=90
  search=1 smoothing=1") instead of "all gains 0, base 0" — **[code-verified]**
- Bug 1 (return to FOLLOWING regardless of searchEnabled): **NOT applied** — the
  SEARCHING branch still early-returns `motorsStop()` when `!searchEnabled`
  BEFORE the line-return check (the stuck-45-min structure is still there; latent
  only because default searchEnabled=true). The `T0 → default timeout` guard is
  also not applied.
- Latest build (arduino-cli, arduino:avr:uno, --warnings more): **0 errors,
  0 warnings, 16792 B flash (52%), RAM 1517 B (74%), 531 B free** — **[HARDWARE-VERIFIED on host toolchain]**
- Full-session diff vs pre-session copy = 10 hunks (4 defaults, 4 g-block,
  banner, comment). r/c/p/d/k/i/b/l/m/t/R/D/z/w/S/Q/F/a/T/n/x/C/W/E/L/X/h intact.

## 2. Hardware findings

- Polarity: white_line=0 — line reads LOW (~170–350), background HIGH (~700–900) — **[HARDWARE-VERIFIED]**
- Calibration saved to EEPROM via E (0xA5 block, CRC8); sensor 0 has the weakest
  MIN/MAX range — **[HARDWARE-VERIFIED]** (supersedes the Nov-6 "sensor 4 weakest" note in stage2 docs)
- baseSpeed < 90 → "GUARD baseSpeed < pwmMin, motors held"; **PWM_MIN = 85**
  (line 60, LAB1 dead-band 75 + margin) — 90 is the lowest reliably-driving base — **[HARDWARE-VERIFIED]**
- Sign of correction correct: robot turns back toward the line — **[HARDWARE-VERIFIED]**
- Loop 20 ms, over=0 with trace off — **[HARDWARE-VERIFIED]**

## 3. Tuning log (kp / kd / base / smoothing → result)

| kp | kd | base | smoothing | result |
|----|----|------|-----------|--------|
| 0.3 | 1.0 | 90 | off | fast zigzag, DRY outputs saturating ±255 — **[HARDWARE-VERIFIED]** |
| 0.2 | 0.3 | 90 | off | zigzag — **[HARDWARE-VERIFIED]** |
| 0.2 | 0.3 | 90 | on | follows straights, loses line on curves — **[HARDWARE-VERIFIED]** |
| 0.25 | 0.3 | 90 | on | loses the line on the curve — **[HARDWARE-VERIFIED]** |
| 0.25 | 0.8 | 90 | on | loses the line; at start spun left, ignored the line — **[HARDWARE-VERIFIED]** |

- kp=0.12 runs: invalid/inconclusive — excluded.

## 4. Known issues

1. Boot CONFIG sometimes showed kp=0.000 kd=1.000 base=0 — matches no source
   version ever on disk; most likely a **stale uploaded hex**. Always re-upload
   and send `C` after boot. (Observation [HARDWARE-VERIFIED]; root cause [UNVERIFIED].)
2. Several `g`/`y` confirmation aborts (CR/LF swallowed the window) — **[HARDWARE-VERIFIED]**;
   y-confirmation since removed (countdown), so this class of failure is gone.
3. Robot sometimes starts in SEARCH and spins left, ignoring the line.
   - `g` reset check (code-read): state ✓ reset, searchFoundCount ✓ reset,
     searchStartTime ✓ reset (elapsed is always millis()−searchStartTime, no
     separate counter), searchTimedOut/crossingUntilMs ✓ reset. lastSeenSide is
     NOT reset by `g`, but it is recomputed from the current error sign at every
     entry into SEARCHING (lost branch) — stale side cannot steer a fresh search.
     → **No fix needed in `g`.**
   - Probable real cause [SIM desk-derived]: during SEARCH, on a good reading the
     code only counts `searchFoundCount++` but never stops the spin; the strip
     sweeps across the line in 1–2 ticks (20 ms), so foundReadingsNeeded=3
     CONSECUTIVE readings never accumulate — foundCount resets to 0 each pass,
     robot spins past the line until timeout.
   - Smallest proposed fix (**diff only, NOT applied**):
     ```diff
     *** Update File: lab2_stage6_track/lab2_stage6_track.ino
     @@ SEARCHING branch, good-reading path
          if (!lineLost(e)) {
     +      motorsStop();   // hold on the line so N consecutive readings can accumulate
            searchFoundCount++;
     ```
4. Docs out of date with the code — need updating:
   - `school_starting_point.md`: recommended values still kp=0.3 ki=0 kd=1 b=90
     smoothing=false → change to sketch defaults kp=0.25 kd=0.3 smoothing=on (ki=0, b=90);
     add the §3 tuning-log rows above.
   - `stage3_start.md`: pre-flight "C should show kp=0.300, kd=0, base=90" and the
     "send g. Within 2 seconds, send y" procedure → new C expectations (0.250/0.300/90/smooth=1)
     and "send g, wait ≤3 s for DRIVING, s cancels"; kp sweep 0.2/0.3/0.5 wording.
   - `next_session_plan.md`: step 7 "send g y" → "send g (3 s countdown)";
     step 8 kp candidates; gotcha "bare d toggles dry-run" still valid.

## 5. Next session checklist

1. Reflash latest build (16792 B), send `C` → expect kp=0.250 ki=0.0000 kd=0.300
   base=90 iLimit=1000.0 smoothError=1, search enabled=1
2. Send `p` and `Q` before `g`: expect lost=0 (e within ±2000), state=FOLLOWING
3. Send `R`, run a straight, then a curve, send `s`, then `D` — paste full dump
4. Read the dump: e flipping sign repeatedly = P too high (oscillation);
   e large with the SAME sign = P too low or base too high (understeer/escape)
5. Then: line-search tests (lost left, lost right, end of line), full track both
   directions, low-battery check (compare b60 behaviour), crossing behaviour
6. Decide on the SEARCH-spin fix from §4.3 (test the hold-on-line diff)
7. Paperwork still to produce: observation notebook (D5), library explainer (D6),
   demo Q&A cheatsheet (D7), track diagram (D9)
