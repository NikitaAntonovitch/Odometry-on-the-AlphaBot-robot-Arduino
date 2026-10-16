# Stage 3 report — straight-line controller for a given distance

> Fill-in skeleton. Sections map to the guide (p.19–22) and checkpoint 3 (p.22).
> The design sections are pre-drafted with the actual constants from the sketch;
> the measurement sections take their numbers from MEASURE_STAGE3.md.

## 1. Design: two controllers, two jobs — [guide p.19]

The straight-line controller is two independent parts acting on the same motors:

**Distance controller = speed profile generator with a stopping condition, NOT a
proportional controller.** Implemented in `controlStep()`:
- progress = mean of the two wheel odometries since motion start (encoder truth,
  so the ramp is position driven, not time driven)
- remaining = target − progress; **stopping condition**: remaining ≤ STOP_TOL_MM
  (10 mm = 2 counts — the quantisation floor derived in the stage 2 report, sec. 8)
- speed profile: cruise V at PWM_CRUISE=130 inside the linear region (stage 2
  characteristic), linear ramps on both ends: ACC_MM=60 at the start (no pwm
  step → no slip at launch), DEC_MM=60 before the target (run-on shrinks to a
  brake-sized stop). The profile floor is PWM_MIN=45 = stage 1 dead band + margin,
  so the motors never enter the dead band while closing in.
- final stop is `brake()` (dynamic braking): stage 1 measured coast run-on as the
  larger of the two, and after the ramp the speed is already at the floor, so the
  residual run-on under brake is within the tolerance.

**Symmetry controller = proportional on the wheel odometry difference** (the one
the assignment explicitly requires):
- err = (dL − dR) · dir, in mm; correction = K_SYM · err, in PWM units, applied
  differentially (left −corr, right +corr)
- K_SYM default 0.6 PWM/mm; correction clamped to ±CORR_MAX=30 so a wheel is
  never driven below the dead band by the correction (the correction-vs-dead-band
  trade-off from the guide, resolved by clamping + documenting)
- why P on the difference: the ~7 % wheel asymmetry (stage 2, right faster) acts
  as a constant disturbance; a P controller on the difference drives that
  disturbance's effect to a small constant offset rather than a growing heading error

## 2. Design: the ramp trade-off — [guide p.20]

The guide's note: during the ramp PWM is close to the dead band, so the headroom
for the symmetry correction is small. In this implementation the profile floor is
PWM_MIN=45 and the correction clamp is ±30, so during the ramp the corrected pwm
can reach 45−30=15 — below the dead band. Consequence: at the very start a wheel
that is "ahead" may stall briefly rather than the robot turning sharply. Observed
in sim as a slight initial heading wobble at high gains; the ramp length (60 mm)
is short enough that this phase is over quickly. Longer ramp = smoother start but
worse early heading correction — measured in section 3 of the worksheet.

## 3. Design: control period and loop budget

- CTRL_MS = 20 (50 Hz). Encoder pulse rate at cruise ≈ 30 counts/s per wheel
  (302 mm/s ÷ 5.109 mm/count) — one edge every ~33 ms, far slower than the control
  period, so no pulses are missed and every control step sees fresh odometry
- one control step is a handful of integer/float ops + two motor writes — well
  under 1 ms at 16 MHz; the rest of the loop is free for serial parsing
- telemetry (when on) prints ~45 chars per 100 ms ≈ 450 B/s < 9600 B/s link budget;
  the guide's warning about telemetry competing with control is answered by the
  10 Hz rate and the buffered Serial writes (tx never blocks at this rate)

## 4. Design: command interface (early protocol decisions, formalised in stage 5)

- `F<mm>` / `B<mm>`: direction carried by the letter; a negative number is taken
  as its absolute value (rule documented, matches the guide's stage 5 example)
- out-of-range (5…5000 mm): rejected with one `err:` line, no motion — the guide's
  "behaviour defined for an out-of-range argument"
- parsing is a non-blocking char state machine (no Serial.parseFloat timeout);
  a number sent without terminator flushes after 50 ms of silence; a bare letter
  is rejected after 300 ms — no default drive, so a typo cannot move the robot
- result line after each motion: `done cmd=±T.mm got=X.Xmm err=Y.Ymm L=n R=m`
  — commanded vs achieved (odometry), per-wheel counts; the only print during a
  drive is the optional telemetry stream (`t` toggle, off by default)

## 5. Measurements (from MEASURE_STAGE3.md — fill after the robot session)

### 5.1 Ramp behaviour
- start jerk: ☐ none / ☐ visible; overshoot: ___ mm past the mark at F500
- DEC_MM/STOP_TOL_MM adjusted: ☐ no / ☐ yes, to ___ / ___

### 5.2 Gain tuning (≥ 4 gains, symptoms)
| gain | symptom described |
|---|---|
| 0 | |
| 0.3 | |
| 0.6 | |
| 1.5 | |
| 3 | |

- chosen gain: ___ , because: ___

### 5.3 Lateral deviation, K=0 vs chosen
- mean |offset| at 500 mm: K=0 ___ mm → chosen ___ mm (improvement factor ___)

### 5.4 Symmetry error vs time, two gains
- logs: tel_K03.txt / tel_K<chosen>.txt; the chosen gain pulls the error to ~0
  within ___ mm of travel; K=0.3 leaves a residual of ___ mm (steady-state offset
  from the P-only controller — expected, documented, acceptable because ___)

### 5.5 Controller error vs scale error — [guide p.21]
| commanded | odometric error | scale error |
|---|---|---|
| F200 | | |
| F500 | | |
| B500 | | |

- conclusion: controller error ≈ ±___ mm (consistent with the 2-count tolerance
  + brake run-on), scale error ___ % → k correction: ☐ not needed / ☐ applied,
  K_MM → ___

## 6. Consequences for stage 4

- the straight-line block is reusable as-is for the straight segments between
  turns; the turn controller will reuse the same profile shape with progress
  redefined as wheel-path-along-the-arc
- the symmetry controller (difference of distances) does NOT apply to spin turns —
  there the requirement is equal magnitudes, opposite signs; pivot mode needs
  only one wheel driven (guide p.24)
- heading error measured after a straight drive feeds the stage 6 phase-1 route
  correction: if the chosen gain holds ≤ 2° over 500 mm, route turns of ≥ 45°
  will not be visibly spoiled by accumulated straight-line heading error
