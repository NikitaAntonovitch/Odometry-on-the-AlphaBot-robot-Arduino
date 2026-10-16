# ISS Lab1 — Stage 2 Plan: Wheel distance & speed from encoders

> **For Hermes:** planning only. Implementation happens in later turns, task-by-task,
> following the user's no-robot-at-home workflow (host sim first, robot numbers at the lab).

**Goal:** add a signed distance counter for each wheel and a calibrated coefficient
`k` (counts → mm) on top of the Stage 1 motor layer, plus a justified speed
measurement — everything Stage 3/4 control will stand on.

**Architecture:** extend the single-sketch style of Stage 1 into
`lab1_stage2_encoders/lab1_stage2_encoders.ino` (motor layer reused unchanged,
encoder module + serial UI added). Validated first on the host harness
(`_hostcheck/` virtual alphabot extended with an encoder model), then by the
PDF's three hand-turn checks + calibration measurements on the real robot.

**Tech stack:** Arduino UNO C++ (.ino), avr interrupts, existing `_hostcheck`
g++ sim (`sim_robot.h` + `sim_main.cpp`), python for measurement post-processing.

---

## 1. Requirements (extracted from the guide, pages 15–17 + 8/11)

| # | Requirement | Source |
|---|---|---|
| R1 | Signed distance counter per wheel (`volatile`, correct width for 8-bit core) | p.15, p.16 |
| R2 | Atomic read of multi-byte counters in the main loop; the protection must NOT re-enable interrupts that were already off | p.16 (explicit) |
| R3 | ISR as short as possible: no arithmetic beyond the counter update, no float, no Serial | p.15 |
| R4 | Direction/sign solved by design — sensor is single-channel and can't tell direction | p.15 |
| R5 | Documented compromise for: standing vibration (false counts) vs coasting (real counts after drive off) | p.15 |
| R6 | Design decision: counts/rev = S (one edge) or 2S (both edges), justified via resolution impact on stage 4 | p.15 |
| R7 | `k = π·D / N_rev`; working value from floor calibration (≥1 m, 3 runs + 1 reverse), compared vs wheel-diameter value, difference explained | p.16 |
| R8 | Speed measurement method chosen + justified (counts-per-window vs period-between-edges) | p.16 |
| R9 | PWM→speed characteristic per wheel, ≥5 PWM values; cruising operating point + wheel asymmetry noted | p.16 |
| R10 | Three pre-calibration checks: single-channel counting, counts per 10 revs, sign fwd/back | p.16 |
| R11 | Variant A (interrupt) vs B (polling) — choice + justification (max pulse rate vs longest loop pass) | p.17 |
| R12 | Report/code: derive k with mean+spread, plot both characteristics + asymmetry | p.16 |
| R13 | Before stage 2: slot count S on BOTH discs (not given in docs), verified electrically (1 wheel rev = predicted counts) | p.8, p.11 |

## 2. Current context / assumptions

- Stage 1 done: `lab1_stage1_motor_test/lab1_stage1_motor_test.ino` (motor layer
  `motorL/motorR/coast/brake`, modes 1–6, `SIGN_L/SIGN_R`, serial UI style:
  one char in → one copyable line out). 26/26 host sim PASS.
- User has NO robot at home → all logic validated on `_hostcheck/` (mock
  `Arduino.h` + `sim_robot.h` virtual robot with pose/wheel state). Robot-only
  numbers (S, D, k, dead band under load) measured at the lab.
- Constraints: ATmega328P, 16 MHz, 2 kB RAM, INT0=D2 (CNTL left), INT1=D3 (CNTR)
  are the only external interrupts and are reserved for encoders.
- Pins already defined in the stage-1 skeleton: `PIN_CNTL = 2`, `PIN_CNTR = 3`.
- Build/sim command (PATH fix for the msys64 clash):
  `PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h -x c++ <sketch>.ino _hostcheck/sim_main.cpp -o _hostcheck/sim.exe`

## 3. Design decisions (to be justified in the report — plan takes a position)

| Decision | Position | Why (report argument) |
|---|---|---|
| Variant A vs B | **A — interrupts (CHANGE edge on both)** | Stage 6 adds protocol parsing + controller to the loop; a long pass (Serial print) must not lose pulses. INT0/INT1 are otherwise unused. Variant B's max-loop-time budget is computed in the report anyway as the comparison argument. |
| Edge mode | **CHANGE → N_rev = 2S** | Resolution is the bottleneck (guide says so); 2× counts/rev halves `k` and directly doubles turn accuracy in stage 4. Cost: ISR runs 2×, still trivial. |
| Counter type | **`volatile int32_t` per wheel** | 8-bit core: multi-byte reads tear without atomic protection; int32 covers any realistic distance (±2·10⁹ counts × few mm). |
| Atomic read | **save SREG + cli/read/sei-restore** (`util/atomic.h` ATOMIC_BLOCK as the portable alternative) | ATOMIC_BLOCK restores prior state → satisfies R2's "must not enable if disabled". Report explains the tear risk on int32. |
| Sign (R4) | **sign = direction last commanded to that wheel** (stored when motorL/motorR run) | Sensor can't provide it; commanded direction is the best available proxy. |
| Vibration vs coasting (R5) | **counting window after drive-off**: edges are accepted while (a) the wheel is commanded to move, or (b) within T_coast ms after the last non-zero command (T_coast from the stage-1 run-on measurement, ~2 s, and coasting decay confirms it); outside the window the counter freezes | Explicit compromise: coast run-on distance still counted; standstill vibration ignored. Discussed as the guide demands. |
| ISR body (R3) | `isrL(){ if (dirL>=0) cntL++; else cntL--; }` — nothing else | Conversions to mm live in the main loop. |
| Speed method (R8) | **counts per time window** (e.g. 100–200 ms window evaluated in the loop) | Speed is only needed for the characteristic (R9); position control (stage 3/4) works on position error and needs no speed. Window method is simplest; low-speed noise is irrelevant at the PWMs we characterize. Period method analysed in the report as the rejected alternative. |
| k (R7) | compute `k_geom = π·D/2S` from calliper D, but adopt calibrated k from floor runs | Guide explicitly wants the calibration value + explanation of the difference (slip, D uncertainty, quantisation). |

## 4. Step-by-step plan

### Task 1 — Extend the virtual robot with an encoder model (`_hostcheck/sim_robot.h`)
**Files:** modify `_hostcheck/sim_robot.h`
- Add per-wheel encoder emulation: `S` slots (configurable, e.g. 20), disc on the
  wheel shaft (1:1 with wheel — guide p.8), square-wave edges = 2 per slot period.
- In `stepPhysics(dtMs)` accumulate each wheel's travelled mm and emit edge
  events; expose helpers: `sim::setEncoderCounts(int32_t,int32_t)`,
  `sim::int32 counters visible via the sketch API`.
- Vibration hook: `sim::jitterWheel(ms)` producing sub-threshold disc wiggle.
- Acceptance: `sim` still compiles; existing 26 stage-1 tests unchanged PASS.

### Task 2 — New sketch skeleton: `lab1_stage2_encoders/lab1_stage2_encoders.ino`
**Files:** create `lab1_stage2_encoders/lab1_stage2_encoders.ino`
- Copy the stage-1 motor layer + UI shell unchanged (program name greeting line —
  closes the stage-1 checkpoint gap noted earlier).
- Add encoder module per §3: counters, dirL/dirR capture in motorL/motorR,
  `isrL/isrR`, `attachInterrupt(..., CHANGE)` in setup, `encoderMM(w)` (atomic
  read + k), `zeroEncoders()`.
- Serial UI additions (same one-char → one-line style):
  `e` = print counts + mm both wheels · `z` = zero · `s` = speed mm/s per wheel
  (window method) · stage-1 drive keys reused (1/2/3/4/5/6/+/-/c/b/h).

### Task 3 — Sim scenario `sim_main.cpp`: pre-calibration checks (R10)
**Files:** modify `_hostcheck/sim_main.cpp`
- T1 single channel: rotate left wheel only → left counter rises, right == 0.
- T2 counts/rev: 10.0 wheel revolutions → counter == 10·2S ±0 (or documented 1-edge value).
- T3 sign: fwd drive → positive mm, reverse → negative.
- T4 coast: cruise then `c` → counter keeps growing during run-on, freezes after T_coast.
- T5 vibration: standstill + `sim::jitterWheel` with drive off → counter unchanged.
- T6 atomic read: hammer the counter from "interrupt" while loop reads — mm values never torn (host can't truly tear; test asserts the guard code path exists via review, marked as design-review item).
- Acceptance: `sim.exe` → all PASS (stage-1 tests still green).

### Task 4 — Sim scenario: speed window sanity (R8/R9 support)
- Drive at pwm 30/60/90 → reported mm/s within ±10 % of sim true speed at each PWM;
  verify window (200 ms) yields ≥ 5 counts per window at lowest cruising PWM
  (the guide's "work out counts per window" argument, checked numerically).

### Task 5 — Measurement worksheets for the lab session (hardware only)
**Files:** create `lab1_stage2_encoders/MEASURE_STAGE2.md` (checklist the user fills at the lab)
- S slots both discs (visual + electrical 1-rev verification) — R13.
- D wheel diameter (calliper).
- k calibration: ≥1 m marked section, 3 fwd runs + 1 reverse, counts recorded.
- PWM→speed: 5 PWM values per wheel (stand), speed from `s` command.
- Asymmetry table + dead-band-adopted value from stage 1.

### Task 6 — Data processing `tools/stage2_process.py`
**Files:** create `tools/stage2_process.py` (host-side, python; reads pasted CSV of counts/distances)
- k mean + spread (reject >2 % spread per guide), compare vs k_geom, print
  explanation placeholders; plot PWM→speed per wheel + asymmetry % (matplotlib).
- Acceptance: run on dummy data produces the two report plots + k table.

### Task 7 — Report section skeleton
**Files:** create `lab1_stage2_encoders/REPORT_STAGE2.md`
- Headings matching R1–R12: variant choice, edge mode, sign solution,
  vibration/coast compromise, atomic-read , speed method choice,
  k table + plots, asymmetry, conclusion for stage 3 (stopping tolerance ≥ 1–2 counts).

### Task 8 — Final gate
- `sim.exe` full suite PASS (stage-1 26 + stagjustificatione-2 new ~10 checks).
- Sketch compiles for UNO at the lab (user flashes; memory: Arduino IDE path).
- Checkpoint-2 items R10/R7/R9 executed on hardware, numbers into worksheets,
  adopted k written into the sketch as `static const float K_MM_PER_COUNT`.

## 5. Files likely to change
- create `lab1_stage2_encoders/lab1_stage2_encoders.ino`
- create `lab1_stage2_encoders/MEASURE_STAGE2.md`, `lab1_stage2_encoders/REPORT_STAGE2.md`
- modify `_hostcheck/sim_robot.h`, `_hostcheck/sim_main.cpp` (encoder model + scenarios)
- create `tools/stage2_process.py`
- untouched: `lab1_stage1_motor_test/` (keep as the stage-1 artifact)

## 6. Tests / validation
1. Host: `PATH="/c/msys64/ucrt64/bin:$PATH" g++ -std=c++17 -I_hostcheck -include _hostcheck/Arduino.h -x c++ lab1_stage2_encoders/lab1_stage2_encoders.ino _hostcheck/sim_main.cpp -o _hostcheck/sim.exe && _hostcheck/sim.exe` → all PASS.
2. Regression: stage-1 checks inside the same suite stay green.
3. Hardware (lab): T1–T3 hand checks, k calibration spread ≤ ~2 %, characteristic monotone above dead band.

## 7. Risks / tradeoffs / open questions
- **Sign from commanded direction fails while pushed by hand** → counts lie; acceptable: assignment only drives autonomously. Documented in report.
- **T_coast guess before run-on measured** → make it a named constant, set from stage-1 numbers at the lab.
- **CHANGE edges double ISR load** — negligible (few hundred Hz max), computed in report (R11 argument).
- **Open question:** does the stage-2 sketch keep the interactive mode keys (1–6) or shrink to distance-only? Plan: keep them — the speed characteristic needs open-loop drive anyway.
- Slot count S unknown until lab → sim uses configurable S (default 20, marked TODO).
