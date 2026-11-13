# Next Hardware Session — Checklist (LAB2, 2026-11-13 prep)

Prepared WITHOUT the robot. All "expect" lines come from code [DEFAULTS] or
host tests [SIM]; nothing here is a hardware claim.

## 0. Before first drive (10 min)

1. Upload the LATEST build (16960 B / RAM 1517 B). Verify the boot banner prints
   "defaults: kp=0.250 kd=0.300 base=90 search=1 smoothing=1".
2. Send `C` → expect:
   `CONFIG kp=0.250 ki=0.0000 kd=0.300 base=90 iLimit=1000.0 smoothError=1`
   `search: enabled=1 spin=120 timeoutMs=3000 foundNeeded=3`
   (If kp=0.000 shows up → stale hex, re-upload — never tune on a stale build.)
3. `W` warmup calibration — rotate by hand the full 10 s; MIN/MAX ranges all > 400.
   Then `E` to save to EEPROM.
4. Quick sanity: `r` (raw), `c` (cal), `p` (position), `Q` (state=FOLLOWING).
5. Regression check for Bug 1 (optional, 2 min): `S` `S` (searchEnabled=0),
   `F` (force search → SEARCHING, LED on, "SEARCH DISABLED - stopped" once),
   slide robot over the line → must print `->FOLLOW` within ~60 ms.
   Then `T0` → expect "T0 refused - restored default searchTimeoutMs=3000".

## 1. Straight and curve testing (45 min)

- `g` → wait ≤3 s for DRIVING line (`s` cancels, hand ready on `s`).
- Straight: `R` record on before the run, `s` to stop, `D` to dump. Save dump.
- Curve: repeat record-run on a gentle curve; dump.
- Read the dumps by the rule: e flipping sign repeatedly = P too high;
  e large with the SAME sign = P too low or base too high.
- kp ladder on the curve: 0.2 / 0.25 / 0.3 (defaults in place; `k` between runs).
  kd ladder if oscillation: 0.3 / 0.5 / 0.8 (`d`). smoothing `m` toggle A/B once.
- Do NOT raise kd back to 1.0 without a reason — [HW] showed ±255 saturation there.

## 2. Search recovery (15 min)

- With search enabled (`S` shows enabled=1): let the robot lose the line on the
  left → expect spin LEFT (side=L from lastSeenSide), recovery in ____ s.
- Repeat on the right.
- End-of-line: expect spin until timeout (3000 ms), "SEARCH TIMEOUT - stopped",
  robot stays stopped. Then `g` again → must start clean (state reset).
- Watch specifically for "spins past the line" (foundCount resets during sweep) —
  if seen, note it; candidate fix exists (motorsStop on good reading) but NOT applied.

## 3. Full track, both directions + crossing (30 min)

- Forward full loop with recording; note per-section behaviour in notebook §6.
- Reverse full loop (memory-less reversal — expect weaker slalom performance).
- Crossing: verify it holds course through the stripe (u frozen 200 ms at
  threshold=3). [HW-PENDING — never tested]
- If the crossing capture fails: `x` toggles crossing handling — record both
  behaviours.

## 4. Wrap-up (10 min)

- Voltage read; behaviour comparison vs fresh battery (notebook §6 table 6.4).
- Fill every empty cell of `observation_notebook.docx` you can from the dumps.
- Instructor question: is the 3 s countdown `g` acceptable in place of the
  written g+y handshake? (Documented as deviation in tuning_reconciliation.md.)

## Still requiring the robot (summary)

- Any kp/kd/base tuning numbers (all existing ones are [HW] from 09-11 only)
- Search recovery times (A–D situations), spin-past-line confirmation
- Crossing behaviour, reverse direction runs
- Boot CONFIG print confirmation (stale-hex suspicion)
- Bug 1 bench regression (steps in §0.5), T0 guard behaviour
- Sensor 0 contrast re-measure after lens cleaning
