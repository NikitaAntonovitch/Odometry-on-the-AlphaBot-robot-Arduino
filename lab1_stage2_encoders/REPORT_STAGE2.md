# Stage 2 report — wheel distance and speed from the encoders

## 1. Design: counting variant (A vs B) — [guide p.17]

**Chosen: variant A — interrupts (INT0/INT1 on D2/D3, CHANGE).**

Justification with my numbers:
- expected max pulse rate: v_max ≈ 448 mm/s (fastest wheel at pwm 210) ÷ k ≈ 5.109 mm/count
  → ≈ 88 counts/s per wheel, i.e. one edge every ≈ 11.4 ms at the very top of the range
- variant B budget: one loop pass must stay under half the shortest period = ≈ 5.7 ms
- stage 6 loads the loop with protocol parsing plus two controllers, and one result
  line of ~20 chars costs ≈ 21 ms at 9600 baud — nearly 4× the polling budget.
  Variant B would lose pulses exactly when the robot is busy reporting. (The same
  line at 115200 would cost ≈ 1.7 ms, but the link runs at 9600 after the stage-2
  link problems — the interrupt variant removes the dependency on link speed entirely.)
- cost accepted: both external interrupts consumed (allowed — no other peripheral uses them)

## 2. Design: edge mode → counts per revolution — [guide p.15]

**Chosen: CHANGE → N_rev = 2S = 40.**

- S = 20 (counted on the disc, both wheels: L 20 / R 20)
- N_rev = 2S = 40 counts/rev → resolution k = π·D/N_rev = π·65.0/40 = 5.105 mm/count
- why: resolution is the bottleneck of the assignment; doubling counts/rev halves k and
  directly doubles the turn accuracy available in stage 4
- cost: ISR runs twice as often; at max speed ≈ 88 Hz — negligible vs 16 MHz
- electrical verification: 1 hand revolution = 40 counts (predicted 2S = 40) ☑

## 3. Design: the direction problem — [guide p.15]

The sensor is single-channel: the counter rises for either rotation sense. Solution:

**Sign = last commanded direction of that wheel** (`dirL/dirR` set in `motorL/motorR`).

- where the sign comes from: commanded direction (the ISR adds ±1 by dirL/dirR)
- limitation: hand-rotated / free-rolling wheels count with the stale sign — acceptable
  because the robot only moves under command in this assignment; documented as a known
  limitation rather than hidden
- what the sim test does instead: bench mode checks magnitude (raw edges with the drive
  off), driven mode checks sign (fwd +, back −)

## 4. Design: vibration vs coasting — the compromise — [guide p.15]

The two requirements conflict (cut counts → lose run-on; keep counting → vibration counts):

**Chosen: a coast window.** Edges are accepted while the drive is commanded or within
COAST_MS = 2000 ms. After the window the ISR gates edges out and the loop snapshots
the counter.

- window sizing: the run-on decay was not re-quantified in this session; 2000 ms is a
  conservative upper bound — far above the visible (centimetre-scale) run-on at cruise
  speed, so genuine run-on is always counted. The cost of a too-long window is only
  that a wheel pushed by hand within 2 s after a stop would count; the operating
  procedure (`z` before any measurement) removes even that.
- run-on distance is still counted (sim: "run-on still counts inside the coast window")
- standstill vibration cannot reach the counter (sim: "live mode: vibration produces no
  counts" — with an intentionally asymmetric jitter model that nets a false count per
  oscillation, i.e. exactly the case the freeze must stop)
- trade-off stated: a wheel pushed by hand more than COAST_MS after stop is not counted

## 5. Design: atomic read protection — [guide p.16]

- counters are `volatile int32_t`; the ATmega328P is 8-bit → a 32-bit read can tear
- `atomicRead()` saves SREG, `cli()`, copies, restores SREG — interrupts are NOT blindly
  re-enabled: if they were already off on entry they stay off (guide requirement)
- alternative considered: `util/atomic.h` ATOMIC_BLOCK — equivalent; hand-rolled chosen to
  make the SREG-restore requirement explicit

## 6. Design: speed measurement method — [guide p.16]

**Chosen: counts per time window** (SPD_WIN = 500 ms, evaluated every loop pass).

- counts per window at the slowest operating point of interest (pwm 90):
  v ≈ 210 mm/s ÷ k 5.109 mm/count ≈ 41 counts/s → ≈ 20 counts/window
  → resolution ≈ 5.1 mm per count ≈ 10 mm/s per window: acceptable for the
  characteristic (measured 2 s after settling)
- period-between-edges rejected because: needs overflow handling for "no counts",
  jitter-sensitive at the accuracies needed here, and speed is NOT used by the stage 3/4
  controllers (they work on position error) — the characteristic is a one-off diagnostic
- low-speed noise accepted as irrelevant for this use

## 7. Measurements

### 7.1 Slot count and pre-checks — [guide p.8/p.11/p.16]
- S: L = 20 , R = 20 (visual), electrical 1 rev = 40 / 40 counts ☑
- single-channel test: L changed, R stayed 0 ☑
- sign test (driven): fwd +, back − ☑
- full worksheet: MEASURE_STAGE2.md

### 7.2 Calibration of k — [guide p.16]
- section 1000 mm, 3 fwd runs + 1 reverse (table in worksheet). The first attempt
  without a stop block gave an 11 % spread — diagnosed as a stopping-position method
  flaw (the guide's predicted failure mode), fixed by driving gently into a block at
  the finish so every run ends identically
- k_mean = 5.109 mm/count, spread 1.54 % (≤ 2 % required ☑)
- k_geom = π·D/2S = 5.105 (D = 65.0 mm measured)
- difference +0.07 % — explained by: quantisation (±k/2 = ±2.55 mm per reading),
  residual slip during acceleration, D uncertainty (tyre squash under load)
- adopted K_MM = 5.1087 (entered as a constant in the sketch)

### 7.3 PWM → speed characteristic + asymmetry — [guide p.16]
- table + plots: see MEASURE_STAGE2.md and stage2_characteristic.png
- linear region: 50 … 210 pwm (whole measured range, mild sag above 170)
  → cruising operating point chosen: 130 (~300 mm/s, headroom both directions)
- asymmetry: +6.5…+7.3 %, right wheel faster, sign constant across the range
  (stage 3 controller must compensate this — noted; well under the 15 % threshold
  that would point at mechanics)
- battery voltage start/end: not measured (no multimeter available); indirect
  stability evidence: k spread 1.54 % and asymmetry flat across the pwm table
  within one session

## 8. Consequences for stage 3

- stopping tolerance cannot be smaller than 1–2 counts = 5–10 mm — this sets the
  tolerance used by the position controller in stage 3 (chosen: 2 counts ≈ 10 mm)
- run-on coast at cruise is centimetre-scale but was never precisely quantified →
  the controller cannot just cut the drive at the target: it needs an anticipatory
  slowdown (position-driven ramp) plus `brake()` at the end, never a bare drive-cut
- asymmetry ≈ 7 % at cruise → Δs ≈ 70 mm per metre between the wheels; with track
  b ≈ 0.15 m (rough estimate, properly calibrated in stage 4) that is ≈ 0.45 rad of
  heading error per metre → lateral drift of order 0.1–0.2 m without control.
  The symmetry controller works exactly on this odometry difference, so it is
  expected to cancel most of it — to be measured in stage 3 (lateral deviation,
  zero gain vs chosen gain)
