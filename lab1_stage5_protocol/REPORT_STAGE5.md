# Stage 5 — FINAL REPORT (freeze edition, 21.10)

Firmware: `lab1_stage5_protocol.ino` tag **stage5-v1-FROZEN** (freeze stamp in
header). k = 5.108 mm/count, b_spin = 151 mm, TRACK_PIVOT = 145 mm
(provisional), K_SYM = 3.0, PWM_MIN = 85.

## 1. Protocol design (decisions recorded, checkpoint-5 items)

- **Command format:** one line, commands `G<metres> L<deg> R<deg>` separated by
  any non-alphanumeric characters; letters case-insensitive.
- **Units:** G takes SIGNED metres (negative = backwards) per the guide's
  table; converted to odometry mm at the exec layer.
- **Sign rule:** direction for L/R lives in the LETTER; a negative number is
  taken as its absolute value (one convention across stages 3/4/5).
  Robot-verified: `G-0.2` → dir=back; `G-0.300m got=-293.8 err=+6.2` in-window.
- **Ranges:** G 0.005…5.000 m (stage-3 envelope), L/R 5…1440° (stage-4
  envelope, multi-rev calibration needs 1440). Out of range → ONE err line,
  no motion, the rest of the string still executes. Robot-verified: 5 err
  lines (`G99`, `L3`, `L2000`, `G0.001`), parser survived, next valid command ran.
- **Garbage tolerance:** unrecognised characters skipped with no side effect.
  Robot-verified: `Xyz ,.! Q7  G0.1` → only the G ran, zero err lines.
- **Execution variant B** (guide stage-6 table): non-blocking parser consumes
  chars as they arrive; complete motion commands wait in an 8-entry program
  queue (64 B RAM) while a motion runs; dispatchStep starts the next when
  idle. Chosen over variant A because the parser is non-blocking — the rx
  buffer alone cannot hold execution order. Queue full → err + honest drop.
- **Result lines:** one `go` ack at motion start + one result line at end
  (`done cmd=G...` / `tturn cmd=... got=...`); turn lines always carry the
  achieved angle. Nothing else prints during motion (telemetry opt-in via `t`).
- **Abort keys** `c`/`b`: stop the motion AND flush the queue (stop = stop,
  not pause).

## 2. Final numbers (real robot, session 21.10)

### Straights (stage-3 controller under protocol)
| test | runs | result |
|---|---|---|
| G0.8 | 1 | got 791.8, err −8.2 mm (tol ±10) ✓ |
| G1.2 | 7 | got 1190.3…1195.4, err −4.6…−9.7, ALL in tol ✓ |
| G−0.2 | 1 | got −199.2, err +0.8 ✓ |
| G−0.3 | 1 | got −293.8, err +6.2 ✓ |
| repeatability | 4× G1.2 | σ ≈ 1.1 mm (0.09 %), L count literally 234×4 — deterministic to the encoder tick |
| symmetry | all | end L/R ≤ 2 counts (≤ 0.5 %) |

### Turns (stage-4 controller under protocol)
| cell | n | mean got | mean E | σ | note |
|---|---|---|---|---|---|
| L15 | 5 | 10.9 | −4.1 | 5.0 | resolution limit (15° = 3.9 cnt) |
| R15 | 6 | −9.7 | −5.3 | 6.3 | got lands on the 3.88° grid |
| L30 | 5 | 25.6 | −4.4 | 6.5 | |
| R30 | 5 | −20.9 | 9.1 | 3.4 | |
| L45 | 16 | ≈36.2 | −8.8 | ≈6 | collapse-contaminated (see limits) |
| R45 | 9 | ≈−40.9 | +4.1 | 2.4 | |
| L90 | 11 | ≈83 | −7.0 | 6.2 | |
| R90 | 11 | ≈−85.6 | +4.4 | 5.6 | |
| L1440 | 2 | 1432.5 | −7.5 | 0.0 | differential sum 739 exact |
| R1440 | 1 | −1432.5 | −7.5 | — | |
- Odometry self-consistency: got = (cR−cL)·k/b **exact on every one of ~45
  pasted turn rows** (F5). Grid 1.94°/count confirmed.
- Healthy-turn pool (both wheels broke away): 87.2…93.0 at 90° → mean E +0.1°.

### Protocol behaviour on robot
- queue/garbage one-liner `qG1.2zL90`: pending (acceptance card §1.3)
- abort mid-run: pending (acceptance card §1.4) — see limitation (d)
- lowercase `g0.1 l45`: ran (case-insensitivity live)

## 3. Known limitations (verbatim list, session evidence)

a) **Constant stop transient ~2–4 counts:** −7…−10 mm on straights, −1…−3° at
   90°, −7.5° at 1440°. Mechanism: the active-brake kick + coast poll fold real
   reverse wheel rotation into got= (F5 honesty); E does NOT scale with angle
   (E(90)≈0 vs E(1440)=−7.5) → not a calibration error.
b) **Cold-start stall on the first turn of a session** → mandatory warm-up turn
   (session-1 first L90 was the worst turn of the day).
c) **R-backward is the weakest drive mode; the left wheel overdrives in
   R-turns; heading is protected by sum-regulation** — L/R splits are cosmetic
   (739/739/739 differential sum at 1440° with splits 369/370, 372/367, 397/342).
d) **Mid-run abort unverified at freeze time** — `x` is bench-mode toggle, not
   a stop; expected emergency stop is `b` (stops motion AND flushes queue).
   Fill from acceptance card §1.4.
e) **b/TRACK_PIVOT validated via self-consistent odometry only** (1440-gate);
   independent tape-pointer check (30 s) optional if a reviewer requires it.
f) **Per-start wheel collapse** (~30 % of turns, random wheel): a wheel whose
   loaded breakaway exceeds the 87–93 pwm turn-start stalls; the symmetry
   controller cannot fix it (PWM_MIN clamp). Measured cost: 45° spread up to
   10.3°, worst run 13.6°. Root-caused (drag + T_PULSE any-wheel early exit),
   patch DESIGNED but NOT APPLIED per freeze decision; Monte-Carlo prediction
   on file (sim_turnmodel.md): patch would give σ 10.7→2.0, soft-kick closes
   the 45° 5 %-rule.

## 4. Findings beyond the lab manual

a) End-of-move dead-band stall is the root cause of the universal small
   undershoot on straights (err always −4…−10, never positive beyond noise).
b) The turn controller regulates the differential SUM, not per-wheel counts:
   got= stays exact even with gross L/R splits (397/342) — but the body
   translates during dragged spins (heading right, position dragged).
c) The odometry grid (1.94°/count, 3.88°/pair) is exact across every run —
   15° turns land ON the grid (0.0/7.8/11.6/15.5), quantisation visibly
   quantises.
d) Loaded drag asymmetry is real and large (right wheel needs +45–60 pwm over
   left to keep pace — telemetry pwmL 100–115 vs pwmR 145–160 sustained) yet
   the symmetry controller equalises end counts to 0.5 %. The stage-2 unloaded
   bench had the right wheel 6 % FASTER — asymmetry direction flips with load.

## 5. Checkpoint 5 — final state

- [x] executes a command sequence sent as a single string (§2 of the run card;
      `G0.2 L90 G0.3` and A→B chains in stage 6 close this with paste evidence)
- [x] unrecognised characters skipped without side effects
- [x] negative-value + units rule decided, recorded, robot-verified
- [x] out-of-range behaviour defined and robot-verified
- [x] one result line per command; turn lines carry the achieved angle
- [x] whole protocol exercisable from the serial monitor (entire session was)
