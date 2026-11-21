# D7 — Demo Q&A Cheat Sheet (1 page)

Structure per answer: what I did → what value → why (reason + observation).
Numeric blanks `____` = fill at school. Never quote a number without its run log.

REVISED 2026-11-21 (teacher fix): sensor input = AnalogRead + readCalibrated
ONLY; own weighted average; lost line by SUM, not sentinels. readLine is no
longer part of the design — if asked, say why (see Q4 soundbite).

---
**Q1. Why does kp have the value it has?**
A: I sized kp from the max-correction formula kp = maxCorrection / errorAtFullCorrection
(≈40/1000 conservative, ≈120/1000 with wheel reversal) and then tuned live from the
sketch default kp=0.25. At kp=____ the robot ____ (zigzags / cuts curves / follows),
so I settled on kp=____. Backing observation: the recorded trace — e flipping sign
every few ticks means kp too high; e staying large with the same sign means kp
too low or baseSpeed too high.

---
**Q2. Why does ki have the value it has?**
A: ki=0. The line follower crosses zero error many times per run, so integral action
adds windup risk (I clamp it anyway with iLimit=1000 and pidReset on every
FOLLOW↔SEARCH transition) and the simulator showed no improvement on long curves
[SIM]. At school I checked ki=____ and saw ____, which confirmed/contradicted
that — final answer: ____. (If the instructor asks for a nonzero ki: only justified
if a long constant-radius curve shows a steady offset.)

---
**Q3. Why does kd have the value it has?**
A: kd damps oscillation; I raised it until the zigzag stopped. kd=1.0 made the DRY
output saturate at ±255 on the real robot [HW], kd=0.3 with smoothing on followed
straights [HW], ____ was the final value. Trade-off I observed: larger kd amplifies
sensor noise between ticks (D of the raw error); smoothing (m command) trades
response sharpness for noise.

---
**Q4. How do you detect a lost line, and why did you choose that search strategy?**
A: I do NOT use the library's readLine() sentinels. I read the strip with
readCalibrated() (the library's intended scaled output), correct the polarity
decided by MEASUREMENT during calibration (dark line → invert 1000−v), and compute
my own weighted average: position = Σ(i·1000·vi)/Σvi over all 5 sensors — ONE
number, error = position − 2000. Lost line = the SUM of the corrected values is
below a threshold (default 250, tuned on hardware: 'p' shows the sum over line vs
background): that means no sensor sees the line, and no fake position is produced.
On a lost tick the code keeps the last valid error, so the robot spins toward the
side where the line was LAST seen, needs N=____ consecutive good readings before
committing (a spinning strip sweeps across the line by accident), and stops after
____ ms (never 0) — safe "end of line" behaviour. If asked why not readLine: its
lost-line report is a −1/4001 sentinel mixed into the position return value — one
variable carrying two meanings; my version returns the position and a separate
visibility flag, which I can unit-test (host tests 9–15 [SIM]).

---
**Q5. What does the controller do at a crossing, and why?**
A: When ≥ CROSSING_THRESHOLD(=3) sensors have corrected values ≥ 300 (the
transverse stripe puts the line under several sensors at once, which would drag
the weighted average sideways), the sketch freezes the last good control output
(lastGoodU) for CROSSING_HOLD_MS=200 ms and holds course, then resumes. Observed
on the crossing section: ____. [HW-PENDING — not yet tested]

---
**Q6. What happens when one coefficient is changed?**
A: Raising kp → faster correction but oscillation risk (observed: fast zigzag at
kp=0.3 with kd=1.0 [HW]). Raising kd → damps oscillation but amplifies noise
(observed: saturation at kd=1.0 [HW]). Adding ki → removes steady offset on long
curves but winds up on lost-line segments unless reset (we pidReset() on every
FOLLOW↔SEARCH transition). Live numbers: ____.

---
### One-line facts to memorise (all [HW] unless marked)
- Position = own weighted average over readCalibrated values; error = position − 2000.
- Polarity (lineDarkFlag) is MEASURED at every calibration: mean(calibratedMin) vs
  mean(calibratedMax); dark line → invert 1000−v. Saved to EEPROM with 'E'.
- Lost line = corrected SUM < lineLostSumMin (default 250 — NEEDS HW TUNING via
  'p' + 't<val>'); lost tick returns the last valid error, never a fake position.
- PWM_MIN=85; baseSpeed=90 is the lowest driving speed; GUARD stops motors below it.
- Loop 20 ms (50 Hz), period absorbed into ki/kd; measured over=0 [trace off].
- Calibration: rotate the robot, full 10 s, every sensor must see line AND background.
- Bug 1 fixed + T0 guard: state exits SEARCHING even when search disabled; host tests 15/15 [SIM].
- g command: 3 s countdown, s cancels — DEVIATION from the written g+y handshake,
  pending instructor OK. Say this proactively if asked about safety design.
