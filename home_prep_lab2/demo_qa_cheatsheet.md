# D7 — Demo Q&A Cheat Sheet (1 page)

Structure per answer: what I did → what value → why (reason + observation).
Numeric blanks `____` = fill at school. Never quote a number without its run log.

---
**Q1. Why does kp have the value it has?**
A: I sized kp from the max-correction formula kp = maxCorrection / errorAtFullCorrection
(≈40/1000 conservative, ≈120/1000 with wheel reversal) and then tuned live from the
sketch default kp=0.25. At kp=____ the robot ____ (zigzags / cuts curves / follows),
so I settled on kp=____. Backing observation: the recorded trace — e flipping sign
every few ticks means kp too high; e staying large with the same sign means kp too
low or baseSpeed too high.

---
**Q2. Why does ki have the value it has?**
A: ki=0. The line follower crosses zero error many times per run, so integral action
adds windup risk (I clamp it anyway with iLimit=1000) and the simulator showed no
improvement on long curves [SIM]. At school I checked ki=____ and saw ____, which
confirmed/contradicted that — final answer: ____. (If the instructor asks for a
nonzero ki: only justified if a long constant-radius curve shows a steady offset.)

---
**Q3. Why does kd have the value it has?**
A: kd damps oscillation; I raised it until the zigzag stopped. kd=1.0 made the DRY
output saturate at ±255 on the real robot [HW], kd=0.3 with smoothing on followed
straights [HW], ____ was the final value. Trade-off I observed: larger kd amplifies
sensor noise between ticks (D of the raw error); smoothing (m command) trades
response sharpness for noise.

---
**Q4. Why did you choose that line-search strategy?**
A: When readLine() reports a lost line (−1 or 4001 from the modified library), the
sign of the error tells which side the line was LAST seen on, so the robot spins
toward that side at searchSpinSpeed=____. It needs foundReadingsNeeded=____
consecutive good readings (~20 ms apart) before committing — a single reading is
not enough because a spinning strip sweeps across the line by accident (assignment
warning). After searchTimeoutMs=____ (never 0 — T0 is refused by the sketch) it
stops and waits: safe behaviour at "end of line". Observed recovery time: ____.

---
**Q5. What does the controller do at a crossing, and why?**
A: When ≥ CROSSING_THRESHOLD(=3) sensors see the line simultaneously, the weighted
average would snap toward the transverse stripe, so the sketch freezes the last
good control output (lastGoodU) for CROSSING_HOLD_MS=200 ms and holds course,
then resumes. Observed on the crossing section: ____. [HW-PENDING — not yet tested]

---
**Q6. What happens when one coefficient is changed?**
A: Raising kp → faster correction but oscillation risk (observed: fast zigzag at
kp=0.3 with kd=1.0 [HW]). Raising kd → damps oscillation but amplifies noise
(observed: saturation at kd=1.0 [HW]). Adding ki → removes steady offset on long
curves but winds up on lost-line segments unless reset (we pidReset() on every
FOLLOW↔SEARCH transition). Live numbers: ____.

---
### One-line facts to memorise (all [HW] unless marked)
- white_line=0: line reads LOW (~170–350), background HIGH (~700–900); setpoint = 2000.
- Lost line = error ±2001 (library returns −1/4001); unsigned trap: 65535.
- PWM_MIN=85; baseSpeed=90 is the lowest driving speed; GUARD stops motors below it.
- Loop 20 ms (50 Hz), period absorbed into ki/kd; measured over=0 [trace off].
- Calibration: rotate the robot, full 10 s, every sensor must see line AND background
  (ranges > 400); stationary calibration = linePos pinned at 2000.
- Bug 1 fixed + T0 guard: state exits SEARCHING even when search disabled; host tests 9/9 [SIM].
- g command: 3 s countdown, s cancels — DEVIATION from the written g+y handshake,
  pending instructor OK. Say this proactively if asked about safety design.
