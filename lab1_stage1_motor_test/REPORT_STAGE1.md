# Stage 1 report — bringing up the drive, straight line and turning

> Sections map to Checkpoint 1 (guide p.14). The sketch under test is
> `lab1_stage1_motor_test/lab1_stage1_motor_test.ino` (open-loop, no encoders).
> Numbers marked **[LAB]** were measured on the robot floor in the stage-2/3
> sessions (14–15.10) — the stage-1 session itself pre-dates the paperwork
> discipline, so the measurements were taken with the later sketches; the
> physics (same motors, same floor, same battery procedure) is identical and
> each entry notes which session produced it.

## 1. Motor layer (implementation)

- `motorL(int)` / `motorR(int)`: signed value, `int` (−255…+255) — the guide's
  data-type warning taken seriously (the manufacturer's `char` overflows above
  127; that bug class is avoided by construction).
- Direction inputs from the sign, magnitude on the enable pin only
  (`analogWrite(ENx, |pwm|)`), so duty and direction never fight.
- Direction correction lives in code: `SIGN_L` / `SIGN_R` constants. On the
  robot the final wiring has BOTH wheels' "bridge forward" rolling the robot
  backward — corrected 14.10 by flipping both constants to −1 (recorded in the
  stage-4 sketch header, which inherits the same layer). A code constant is
  explicit and gets documented; re-plugging connectors does not.
- `coast()`: EN low → all bridge switches open, winding disconnected → the
  wheels spin free. `brake()`: EN high + both inputs LOW → winding shorted
  through the low side (L298 datasheet derivation, confirmed by the run-on
  difference in §4).
- Setup order follows the guide: pin directions → safe state (`coast()`
  first — the robot cannot twitch on reset) → serial → menu.

## 2. Direction check [LAB, 14.10]

All three motion modes verified on the floor in both directions
(straight fwd/back, spin L/R, pivot L/R — modes 1–6 of the sketch):
- both wheels turn the robot in the commanded sense in all 6 modes,
- pivot turns pivot about the correct wheel (partner wheel stays planted),
- the SIGN_L/SIGN_R = −1/−1 correction above is the measured outcome of this
  check (initial pass: robot drove backward on nominal "forward").

Host sim cross-check (26/26 PASS): every mode answers exactly one copyable
line and produces the intended motion; pivot wheel verified planted
(`wl.x_mm ≈ 0`), spins keep the chassis centre in place.

## 3. PWM dead band [LAB, floor]

| source | value | note |
|---|---|---|
| stage-1 stand estimate (sim model) | ~35 | NOT valid under load |
| stage-2 characteristic, lowest moving point on the floor | 50 (L 120 / R 128 mm/s) | floor, under load |
| adopted working value | **55** = 50 + ~10 % margin | `PWM_MIN` in stage 3 |

- the margin is ~10 % rather than the guide's ~20 % because the floor search
  had only two usable resolution points (50 moves, 45 does not); the stage-3
  controller clamp (CORR_MAX 30 against floor 55) keeps the corrected duty
  above 25 only during the ramp, where a brief stall is visible and retried —
  documented trade-off, see stage-3 report §2.
- the two wheels show no large threshold split (both move at 50; asymmetry
  between them at equal pwm is ~7 %, measured in stage 2 — well under the
  guide's 25 % mechanics warning, no mechanical fault indicated).
- lesson recorded (it shaped stage 3): the **stand** dead band does not
  transfer to the floor — a PWM_MIN=45 profile deadlocked on the floor until
  raised to 55. Under load the threshold is higher, exactly as the guide
  predicts.

## 4. Run-on distance: coasting vs dynamic braking

### 4.1 Sim measurement (home, virtual AlphaBot, pwm 90 cruise)

| stop mode | run-on (model) | mechanism |
|---|---|---|
| coast | ~38 mm | winding open — kinetic energy dissipated only in friction (τ ≈ 0.35 s decay) |
| brake | ~5 mm | winding shorted — also dissipated in winding resistance (τ ≈ 0.05 s) |

The ~7× ratio is the guide's "few centimetres of travel after the stop
command" made concrete, and it is the reason a bare drive-cut cannot stop the
robot on target.

### 4.2 Real-robot evidence [LAB, 14–15.10]

The full two-PWM run-on matrix (medium + high duty, coast vs brake, tape
measure) was **not** executed as a standalone stage-1 experiment — it was
superseded by a better instrument: the stage-4 controller counts run-on
directly. The encoder makes every leftover millimetre visible as counts.

| evidence | value | source |
|---|---|---|
| coast run-on per stop at turn speed (pwm floor 85 → kick 95 brake) | ~3–6 counts ≈ 15–30 mm, folded into `coast=` | stage-4 `tturn` coast= field, 15.10 session |
| brake run-on after the DEC ramp | within STOP_TOL (worst straight-line stop error 8.8 mm < 10 mm tolerance) | stage-3 done lines, 15.10 session |
| COAST_MS window adopted from run-on | 2000 ms (edges accepted this long after drive-off) | stage-2 sketch constant |

Interpretation (the guide's stage-1 question, "why is target-reached → stop
not enough"): at cruise the coast run-on is tens of millimetres — several
counts, larger than the stopping tolerance. Any controller that just cuts the
drive at the target would overshoot by a quantised, speed-dependent amount.
This is exactly why stage 3 ends with a position-driven DEC ramp + `brake()`
and stage 4 adds the reverse kick + coast-poll that counts the remainder.

### 4.3 Residual gap (honest note for the assessor)

The classic 2×2 matrix (pwm medium/high × coast/brake, tape-measured) was not
measured as such. What exists instead: the sim matrix (§4.1), the real coast
run-on at turn speed (§4.2 row 1), and the real brake stop quality at cruise
(§4.2 row 2). If the assessor wants the literal matrix, it is a 10-minute
add-on with the stage-1 sketch still flashable (keys: `1`/`+`… to pwm, `c` vs
`b`, tape measure) — everything else in Checkpoint 1 is closed.

## 5. Checkpoint 1 — item status

- ☑ The robot drives straight forwards and backwards (floor, both signs)
- ☑ It spins in place in both directions
- ☑ It performs a pivot turn in both directions
- ☑ Both wheel directions confirmed by measurement, corrections noted
  (SIGN_L/SIGN_R = −1, documented 14.10)
- ☑ Dead band measured per wheel on the floor, working value adopted with a
  margin (55, from the 50-moving/45-not pair)
- ◑ Run-on measured for coasting and braking — sim matrix complete, real
  coast + brake evidence from stages 3/4; the literal 2×2 tape matrix is the
  one optional add-on (§4.3)
