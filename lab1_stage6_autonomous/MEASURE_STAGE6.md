# Stage 6 — autonomous A → B drive (acceptance session worksheet)

Sketch to flash: `lab1_stage6_autonomous/lab1_stage6_autonomous.ino` — logic
IDENTICAL to stage5 v1 (guide: "nothing new to write beyond merging stages 1–5"),
re-identified: greeting `stage6 v1: A-B autonomous | G/L/R protocol | k=5.109
spin=151 pivot=145 [prov]`. Serial 9600, Newline.

> **DEADLINE MODE (one session covers stages 5+6):** §0 + stage-5 singles can be
> brief — if the pack and floor allow, run the stage-5 worksheet (§1–§6) FIRST,
> then this file on the same charge. The A→B route below is deliberately simple
> (one turn) so the deadline version still satisfies the guide's "at least one
> change of direction" with margin for the 45° requirement.

## 0. Test area (guide's numbered prep)

1. **A and B on the floor, ≥ 2 m apart, ≥ 1 turn between them.** Layout
   (degrees assume the spin track 151; all turns L):
   - **Option 1 (L-shape, recommended):** A → `G1.0` → `L90` → `G1.0` → B.
     Needs a 1 m + 1 m corner. Route string: `G1.0 L90 G1.0` (14 chars).
   - **Option 2 (deadline-safe):** A → `G1.2` → `L45` → `G1.2` → B. Same 2 m
     straight legs, gentler turn. Route string: `G1.2 L45 G1.2` (14 chars).
2. **Start orientation mark at A** — tape line under the robot's centre axis.
   Without it phase 2 has no chance to reproduce phase 1 (guide's words).
3. **B marker:** pen cross on a paper sheet (deviations read off it at ±5 mm).
4. Charged cells, voltage noted if a meter appears: ______ V (calibration note:
   8.4 → 7.0 V shifts speed >10 % and invalidates ramps/dead band).

## 1. RX-buffer math (guide: "to work out", BEFORE phase 2)

- route string = `G1.0 L90 G1.0` = 14 chars (option 1) / `G1.2 L45 G1.2` = 14
  chars (option 2). With `\n` = 15 bytes.
- UART rx buffer = **64 B** (Arduino core, HardwareSerial). 15 < 64 → the whole
  string fits → sending at once is safe.
- execution variant in the sketch = **B** (8-command program queue), so even a
  string longer than 64 B would still execute whole — the capacity check is
  recorded for the report but variant B makes it non-critical.
- commands per route = 3 ≤ QUEUE_CAP 8 ✓ (note if you extend the route: >8
  queued commands drop with `err: queue full`)

## 2. Phase 1 — establishing the route (single commands, robot decides)

Robot at A on the start mark. Send ONE command, watch, choose the next. Every
command that ends up in the sequence is written below **as it will be sent** —
corrections MERGE (`G1.0` + extra `G0.3` becomes `G1.3`), two same-way turns
add up, a turn between straights cannot be merged (guide's rules).

| step | sent | pasted result line | what I saw | kept in sequence? |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |

- final route string (this exact text goes to phase 2): `____________________`
- string length = ______ chars ≤ 64 ✓
- odometry says the robot is at B; tape says deviation from the B cross =
  ______ mm (phase-1 residual — phase 2 is judged against the SAME tape point)

## 3. Phase 2 — autonomous drive (the acceptance runs)

Carry the robot back to A, ON the start mark, wheels on the tape line. Send the
whole route string from §2 as ONE message. **No other input until it stops.**

| run | pasted full log (go/done/tturn lines) | end point (x,y) mm from A | heading dev (deg) |
|---|---|---|---|
| 1 | | | |
| 2 | | | |
| 3 | | | |

Read end points off the B-cross paper (axes: x = A→B nominal direction per the
route drawing, y = left). Heading deviation: re-aligned tape edge vs start
direction, protractor or the marked circle. Between runs: robot back to A,
exactly on the mark — the guide's "the orientation mark has to be reproduced".

**Abort rule (unchanged from stage 5):** `c` mid-run stops and flushes the
queue — a run that needed `c` is NOT an acceptance run; fix the cause, repeat.

## 4. Phase 3 — processing (feed tools/stage6_process.py)

Enter the three end points into the script (or paste them here and the script
parses): mean shift = SYSTEMATIC component (calibration: k along the route,
track across it), spread (σ of the 3 points) = RANDOM component (resolution +
slip — tuning will not help; guide's table).

| quantity | value | component | fix if too big |
|---|---|---|---|
| mean end-point shift | ______ mm | systematic | mmPerCount (k) re-check |
| lateral mean shift | ______ mm | systematic | wheel track re-check |
| spread σ (3 runs) | ______ mm | random | resolution/slip — document, don't tune |
| odometry-vs-tape route length | ______ mm | calibration error, NOT controller error | |

- why phase 2 differs from phase 1 (write 2-3 lines, guide's table as source):
  ______

## 5. Checkpoint 6 ticks

Functionality (mostly proven by the stage-5 session — tick with its evidence):
- [ ] drive and turn commands each end with a result line (stage-5 §1)
- [ ] whole command sequence from a single string (stage-5 §2 + phase 2 here)
- [ ] unrecognised characters cause no motion (stage-5 §3)
- [ ] out-of-range handled per the adopted rule (stage-5 §5)
- [ ] units + sign rule documented (report sec.2, confirmed by stage-5 §4)

A → B drive (this session):
- [ ] phase 1: route established with single commands, sequence recorded (§2)
- [ ] string length computed vs buffer capacity (§1)
- [ ] phase 2: autonomous drive from one string, THREE times (§3)
- [ ] phase 3: spread measured, error components separated (§4)
- [ ] phase-2-vs-phase-1 difference explained (§4)
