# Stage 2 — measurements taken at the lab (filled, session of 09.10)

Everything below is hardware-only. Sketch flashed:
`lab1_stage2_encoders/lab1_stage2_encoders.ino`. Serial monitor: **9600**
(`SERIAL_BAUD` in the sketch; the link speed was lowered from 115200 after the
stage-2 link problems — the sketch constant is authoritative).
Before power-up: encoder discs fitted, not rubbing; sensors plugged to their own
wheels; cells charged. Battery voltage **not measured** (no multimeter
available) — see the stability note in §5.

## 1. Slot count S (guide p.11 — done FIRST, it feeds everything else)

| wheel | slots counted (visual) | counts for 1 hand rev (electrical, `x` then `z`, turn 1 rev, `e`) | N_rev = counts | agree? |
|---|---|---|---|---|
| L    | 20 | 40 | 2S = 40 | ☑ |
| R    | 20 | 40 | 2S = 40 | ☑ |

- electrical check done in bench mode (`x` key → `bench=on`), drive off, hand turn
- electrical = 2 × visual → CHANGE both edges confirmed
- (half/double failure modes did not occur: disc clean, no vibration double-count)

## 2. Wheel diameter D

D = 65.0 mm (measured across the tyre — exactly 6.5 cm, single value)
→ D_avg = 65.0 mm

## 3. Pre-calibration checks (guide p.16 table)

| test | how | expected | result |
|---|---|---|---|
| single channel | `x`, `z`, hand-turn LEFT wheel several revs, `e` | L changes, R = 0 | ☑ |
| counts/rev | `z`, hand-turn 1 rev, `e` | 1·2S = 40 ± 0 | ☑ 40 both wheels (the 10-rev variant was not run separately; the 1-rev check asserts the same thing) |
| sign of distance | live mode (`x` off), `1` fwd, `e`, then `2` back, `e` | counts rise, then fall | ☑ fwd +, back − (confirmed again by calibration run 4) |

## 4. Calibration of k (guide p.16 — floor marks, section L_sec = 1000 mm)

Robot wheel axis ON the start line; **stop block flush at the finish line** —
each run ends by gently driving into it, so every run stops at the identical
position. (Without the block the first attempt gave an 11 % spread — a
stopping-position method flaw, not hardware; the block fixed it.)

| run | direction | counts L | counts R | k_L = L_sec/cnt_L | k_R |
|---|---|---|---|---|---|
| 1 | fwd | 196 | 194 | 5.102 | 5.155 |
| 2 | fwd | 197 | 195 | 5.076 | 5.128 |
| 3 | fwd | 195 | 196 | 5.128 | 5.102 |
| 4 | reverse | −196 | −197 | 5.102 | 5.076 |

- k = mean of all 8 readings (both wheels trusted: L and R agree within one count)
- k_mean = **5.109 mm/count**
- spread = (5.155 − 5.076)/5.109 = **1.54 %** → ≤ 2 % ☑
- k_geom = π·D/2S = π·65.0/40 = 5.105 → difference **+0.07 %** — explained by:
  quantisation (±k/2 = ±2.55 mm per reading), residual slip, D uncertainty
  (tyre squash under load)
- adopted value → `K_MM = 5.1087` entered in the sketch ☑

## 5. Static PWM → speed characteristic (floor)

For each PWM: settle 2 s, press `s` twice, second reading noted.

| PWM | L mm/s | R mm/s | asymmetry % = (R−L)/avg |
|---|---|---|---|
| 50  | 120 | 128 | +6.5 |
| 90  | 210 | 226 | +7.3 |
| 130 | 292 | 312 | +6.6 |
| 170 | 356 | 382 | +7.0 |
| 210 | 420 | 448 | +6.5 |

- cruising operating point = inside the linear part → picked: **130**
  (~300 mm/s, mid of the measured range, headroom both directions)
- asymmetry 6.5–7.3 %, same sign across the whole range (right wheel faster),
  well under the 15 % mechanics-warning threshold → mechanics fine, the stage 3
  symmetry controller will compensate
- battery voltage after the session: not measured (no multimeter). Indirect
  stability evidence: k spread stayed 1.54 % and the asymmetry is flat across
  the pwm table — the supply did not visibly wander during the session

## 6. Entered into the sketch

```
static const uint8_t  SLOTS   = 20;      // measured
static const float    WHEEL_D = 65.0f;  // measured
static const float    K_MM    = 5.1087f; // calibrated (section 4)
static const uint16_t COAST_MS = 2000;  // conservative bound, see report sec.4
```
