# Stage 4 — report pre-fill from SIMULATED dry-run (15.10)

> Every number below marked **[SIM]** comes from the desk model (b_true 150.9 spin /
> 145.3 pivot, skid +3.3° cruise / +0.5° table, slip σ 0.5°, tape σ 0.3°), NOT from
> the robot. Real numbers from earlier sessions are marked **real**. The real
> session must reproduce each [SIM] block inside the acceptance ranges in §G
> before any [SIM] cell may be quoted as evidence.

## A. Calibrated-parameter block (goes into report §3)

| parameter | value | status |
|---|---|---|
| k | 5.109 mm/count | real (stage 2) |
| TRACK_MM (spin) | 151.0 | real 1440° session + joint fit (s=1.059, c=+4.7); [SIM]-confirmed by gate + closure |
| TRACK_PIVOT | 145.0 | [SIM] — from [SIM] 360° pivot E +15.3/+14.1 → factor 145.1; real 360° run required |
| turn-speed policy | PWM 65 for the accuracy protocol; cruise 85 documented as trade-off | decision (see §G note 3) |
| STOP_TOL_DEG | 0.7 | unchanged |

Naive b = 151×1440/1445.8 = 150.4 **rejected** [SIM evidence]: the +5.8° gap
decomposes into quantisation overshoot (+2.4, 743/744-count landing) + skid
(+3.3) + scale residual (~+1.0) — no room for a scale error. Independent check:
gate L90 E +4.4 = overshoot 1.1 + skid 3.3 + scale 0.05 exactly; b=150.4 would
predict +4.85.

## B. Resolution table at b = 151 (spin) / 145 (pivot)

| mode | 1-count step | effective stop step | stop landing window (tol 0.7°) | counts/wheel 15/30/45/90 | half-step vs 15° |
|---|---|---|---|---|---|
| spin | 1.94° (k/b) | 3.88° (wheels tick in pairs) | −0.7 … +3.2° | 3.9 / 7.7 / 11.6 / 23.2 | ±1.94° = 12.9% |
| pivot | 2.02° (k/b_pivot) | 2.02° (one wheel ticks) | −0.7 … +1.3° | 7.4 / 14.9 / 22.3 / 44.6 | ±1.01° = 6.7% |

Note for the text: pivot's "2× resolution" is vs the spin's PAIRED step
(3.88 → 2.02, ×1.9), not vs spin single counts (1.94 ≈ 2.02). A 15° spin is
~1.9 paired steps — half a step of landing uncertainty is 13% of the command.

## C. Before/after error decomposition (spin, cruise)

| turn | scale term (b wrong) | quant. overshoot | skid | predicted E | observed E |
|---|---|---|---|---|---|
| 15° @ b=160 (real) | +0.9 | ~+1 | ≈0 (ramp-bound, never cruise) | ~+2 | **real ≈ 0** ✓ |
| 45° @ b=160 (real) | +2.7 | ~+1 | part of constant | ~+7 | **real +8** ✓ |
| 90° @ b=160 (real) | +5.4 | ~+1 | +3.3 | ~+10 | **real +12** (Δ2°, noise) |
| 1440° @ b=160 (real) | +86.8 | ~+2.4 | +3.3 | ~+92 | **real +90** ✓ |
| 90° @ b=151 [SIM] | +0.05 | +1.1 | +3.3 | +4.45 | **[SIM] +4.4** ✓ |
| 1440° @ b=151 [SIM] | +1.0 | +0.4…+2.4 | +3.3 | +4.7…+6.7 | **[SIM] +5.8** (mixed 743/744) ✓ |
| 45° @ b=151, cruise [SIM] | +0.3 | +0.9…+2.8 | +3.3 | +4.5…+6.4 | **[SIM] +4.5** ✓ |
| 45° @ b=151, PWM65 [SIM] | +0.3 | +0.9…+2.8 | +0.5 | +1.7…+3.6 | **[SIM] +1.8** ✓ |

Model caveat [SIM]: the binary skid term (3.3 whenever "cruise") over-predicts
15° — a 15° turn is all ramp (ACC 30°) and never reaches cruise, which is why
the real 15° @ b=160 sat at ~0. Treat skid as speed/time-bound, not on/off;
the real table calibrates it.

## D. Accuracy tables

### D.1 Spin, PWM 65 — mean E / spread, deg [SIM]
| angle | L | R |
|---|---|---|
| 15 | +0.90 / 1.42 | +0.76 / 1.35 |
| 30 | +1.04 / 0.87 | +1.12 / 0.91 |
| 45 | +1.52 / 0.85 | +1.52 / 0.95 |
| 90 | +0.76 / 0.93 | +0.78 / 0.98 |

45° combined E = +1.52 ≤ 2.25 (5% rule), worst run +2.1. got= on the 1.94°
grid every rep; measured values off-grid (bimodal 23/24-count landings at 45°,
46/47 at 90° — spreads 0.85–1.4 are the bimodal signature, expected).

### D.2 Pivot, TRACK_PIVOT 145 — [SIM]
| angle | L | R |
|---|---|---|
| 15 | +1.26 / 0.95 | +1.10 / 1.05 |
| 30 | +1.30 / 1.10 | +1.15 / 0.98 |

### D.3 Mode comparison @15° [SIM]
spin +0.83 (σ 1.39) vs pivot +1.18 (σ 1.00) — the earlier real +5° pivot
excess is gone (it was the uncalibrated pivot track, pure scale: 151/145.3
ratio × 360° = +14.1° matches [SIM] pre-change 360° E +14.1/+15.3 exactly).
Pivot advantage shows in SPREAD (1.00 vs 1.39), not mean — say so honestly.

## E. 15° resolution-limit discussion (report §2 filler)

15° = 3.9 counts/wheel spin = ~1.9 paired steps → the stop lands within
±1.94° (12.9%) no matter the tuning; STOP_TOL 0.7° (4.7% of 15°) cannot be
resolved inside one paired step. Workarounds on the table: pivot mode
(2.02° step, ×1.9 finer), lower speed (less skid/run-on — D.1 ran at PWM 65
for exactly this reason), operator loop-closing via got= in the result line.

## F. Track-closure block (report §3 filler) [SIM]

1440° ×3 L + ×3 R at J151, cruise: E(L) +5.9/+5.3/+6.1, E(R) +5.6/+6.3/+5.8;
mean +5.8, σ 0.4, same sign both directions → caster asymmetry not present at
this resolution. TRACK_MM stays 151.0.

## G. REAL-SESSION ACCEPTANCE RANGES (a [SIM] conclusion stands only if real data lands inside)

| # | run | acceptance |
|---|---|---|
| 1 | gate: L90 + L1440 at J151, paste got= lines | got= on 1.94° grid (±0.1°); L90 E +3.5…+5.5; L1440 heading E +4…+8 |
| 2 | 1440° ×3 L + ×3 R, cruise | E +4…+8 both dirs, same sign, σ ≤ 1.5, L/R means within 2° |
| 3 | pivot 360° L+R @ 145 (after reflash) | E −2…+3; got= 359.4 (rem≤0.7 rule) or 361.4 (≥target rule) — records which stop rule the firmware uses |
| 4 | speed gate: L45 @65 and @cruise | @65 E +0.5…+2.5; @cruise E +3…+6 |
| 5 | spin table 8 cells × 5 reps @65 | cell mean E −0.5…+2.5; spread 0.8…2.0; 45° combined ≤ 2.25; got= on-grid every rep |
| 6 | pivot 15/30 × 5 reps × 2 dirs @145 | mean E −0.5…+2.5; spread 0.7…1.6 |
| 7 | mode comparison @15° | pivot spread ≤ spin spread (direction, not 2×) |

Notes: (1) enter TRACK_MM 151.0 / TRACK_PIVOT 145.0, reflash, verify with run 1
before collecting; (2) paste every got= line — per-run decomposition uses the
actual got=, not an assumed parity; (3) the report must show the cruise 45°
number ([SIM] +4.5, real range +3…+6) next to the table-speed result and state
PWM 65 as the adopted turn policy — otherwise the 5%-rule pass looks like
criterion-shopping; (4) 5% at 45° is a self-imposed criterion (the assignment
requires only 45° resolution) — label it as such.
