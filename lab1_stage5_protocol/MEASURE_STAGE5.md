# Stage 5 — measurements to take at the lab (fill this in on the robot)

Sketch to flash: `lab1_stage5_protocol/lab1_stage5_protocol.ino`. Serial: **9600**,
line ending **Newline** (or Both NL&CR — both parse). Command string:
`G<metres> L<deg> R<deg>`, e.g. `G1.5 L45 R-45 G2`.

Run order: §0 pre-flight → §1 singles → §2 sequence → §3 skip/case → §4 signs →
§5 range → §6 queue/abort → §7 checkpoint ticks. **Paste every line the robot
prints** into the tables — the report is filled from them at home.

Expected-value sources (do not treat as [SIM] evidence): straight got window
[cmd−10, cmd−6] mm is a CODE-STRUCTURE fact (stop fires at rem ≤ 10 mm, one
20 ms step ≈ 4 mm at stop speed); turn gates come from the stage-4 §G acceptance
ranges at the fitted track 151. If a 90° turn shows E ≈ +12 again, the flashed
binary is not v1 / constants mismatch — check the greeting, stop, do not tune.

## 0. Pre-flight gate (same as stage 4 — stage 5 rides on its calibration)

- [ ] **Hand-push:** `z` → push straight ~0.5 m → `e`. L = ______ cnt, R = ______
  cnt, R/L = ______ (accept 0.95…1.05). This is the stage-3 formal pair too.
- [ ] **Pack:** charged cells. `err: stalled` with prog 0.0 → stop session, charge
  (row-9 lesson: sag raises breakaway above PWM_MIN 85).
- [ ] **Greeting** on reset: `stage5 v1: G/L/R protocol | k=5.109 spin=151 pivot=145 [prov]`
- [ ] **Floor:** ≥1.5 m clear straight lane + the §2 L-shape marked with tape;
  start heading marked.
- [ ] Serial monitor set to 9600, something in the line-ending box.

## 1. Single commands (robot anywhere, standstill before each)

| # | send | expect (derived above) | pasted lines | tape/protractor | ok? |
|---|---|---|---|---|---|
| 1.1 | `G0.2` | go dir=fwd + done got 190…195 mm | | end ~20 cm from start | |
| 1.2 | `G-0.2` | dir=back, got −190…−195 mm | | back at start ±5 mm | |
| 1.3 | `L90` | tturn got +93.5…+96.0 | | protractor 90…96 | |
| 1.4 | `R90` | tturn got −93.5…−96.0 | | protractor −90…−96 | |
| 1.5 | `L45` | tturn got +48…+51 (assignment's 45°) | | protractor 45…51 | |

## 2. THE checkpoint item — sequence as ONE string

Mark the start point + heading. Type the whole line, press Enter once:

```
G0.2 L90 G0.3
```

Expect 5 lines: go / done / go / tturn / done — motions chained with no operator
action between. String length 13 chars vs 64 B rx buffer (report math, fits).

- pasted log:
- got values: G-leg1 ______ mm, turn ______ deg, G-leg2 ______ mm
- endpoint prediction in the ODOMETRY frame: with the turn's got= θ,
  end = (200 + 300·cos θ, 300·sin θ) mm from start (x = start heading, y = left).
  θ = ______ → predicted ( ______ , ______ ) mm
- tape-measured end point: ( ______ , ______ ) mm → deviation ______ mm
  (accept ≤ 40 mm — residual here is straight-leg lateral error, i.e. the
  symmetry controller's real-world quality)

## 3. Skip rule + case-insensitivity

| # | send | expect | pasted | ok? |
|---|---|---|---|---|
| 3.1 | `Xyz ,.! Q7  G0.1` | junk skipped silently (NO err lines), one G drive runs | | |
| 3.2 | `g0.1 l45` | lowercase = same commands | | |

## 4. Sign rules (the report documents these — confirm they hold)

| # | send | expect | pasted | ok? |
|---|---|---|---|---|
| 4.1 | `L-45` | turns LEFT ~45 (abs rule, letter decides) | | |
| 4.2 | `R-45` | turns RIGHT ~45 | | |
| 4.3 | — | (G negative already covered in 1.2) | | |

## 5. Out-of-range behaviour (err line, NO motion, string continues)

| # | send | expect | pasted | ok? |
|---|---|---|---|---|
| 5.1 | `G99` | err, robot does not move | | |
| 5.2 | `L3` | err (below 5 deg) | | |
| 5.3 | `L2000` | err (above 1440 deg) | | |
| 5.4 | `G0.001` | err (below 0.005 m) | | |
| 5.5 | `G99 L90` one line | err for G99 AND the L90 still runs | | |

## 6. Queue + abort semantics (variant B)

| # | action | expect | pasted | ok? |
|---|---|---|---|---|
| 6.1 | `L90 L90` one line | both turns run; net heading ≈ +7…+11 deg (run-on bias doubles — record, not a fail) | | |
| 6.2 | `G0.5`, then `L90`+Enter WHILE it drives | no preempt: L90 waits; after done, L90 runs | | |
| 6.3 | `G0.9`, mid-drive `c` | stops (mode=stop), coast decay ~1 s; NO queued command runs after | | |
| 6.4 | after 6.3: `G0.3` | robot accepts new commands normally | | |

## 7. Checkpoint 5 ticks (fill from the sections above)

- [ ] executes a command sequence sent as a single string (§2)
- [ ] unrecognised characters skipped without side effects (§3)
- [ ] negative-value + units rule decided and recorded (§4 + report sec.2:
  G signed metres, negative = backwards; L/R abs rule; mm conversion at exec)
- [ ] behaviour defined for out-of-range argument (§5)
- [ ] one result line after each command; turn lines carry got= (§1/§2)
- [ ] whole protocol exercisable from the serial monitor, no PC program (this sheet)

## 8. Optional if time remains

- pivot in the protocol: `m` → `L45` → one wheel planted (tape-check the planted
  wheel), got ≈ +47…+49 — then `m m` back to spin
- `p` mid-motion prints state; `t` telemetry on for one G run (then off)

## 9. Bring home

- this file with every pasted line + tape/protractor numbers
- battery voltage if a multimeter appears (standing note: not measured)
- any `err:` or `coast=` lines verbatim — they feed the report's behaviour notes
