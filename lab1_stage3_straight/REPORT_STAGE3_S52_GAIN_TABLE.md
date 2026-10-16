### 5.2 Gain tuning — Table 1 (REAL data, 14.10 floor session; B500 runs)

> Method: the gain is a runtime constant changed from the terminal with the
> `K<value>` command — no reflash between runs, so consecutive runs share the
> same battery state and the same floor marks. For each run the robot drives
> B500 (500 mm backward) and the **L−R end-count difference is read from the
> done-line counters** (`done cmd=... L=n R=m`); the difference of the wheel
> odometries is the quantity the symmetry controller acts on, so its end value
> is the direct measure of how much wheel-path inequality survived the run.
> Derived columns use K_MM = 5.1087 mm/count and track b = 151 mm:
> 1 count of difference = 5.109 mm of wheel-path difference = 1.94° of heading.

**Table 1 — L−R end difference vs symmetry gain (B500)**

| gain (PWM/mm) | n | L−R end mean (cnt) | spread (cnt) | L−R mean (mm) | heading equiv (°) | symptom observed | verdict |
|---|---|---|---|---|---|---|---|
| 0 (reference) | 2 | 14.0 | 8.0 | 71.5 | 27.1 | visible rightward curve at cruise, no correction fighting the +7 % wheel asymmetry | baseline: controller off, asymmetry fully in charge |
| 1.0 | 1 | 4.0 | 0 (n=1) | 20.4 | 7.8 | curve visibly reduced; no stutter, no weaving | effective; residual 3.5× smaller than K0 |
| 3.0 — **CHOSEN** | 1 | 2.0 | 0 (n=1) | 10.2 | 3.9 | no visible symptoms: straight path, no weaving, no wheel stall, stop normal | best measured equalisation; adopted as K_SYM = 3.0f |
| TODO (4th gain) | — | TODO (not measured) | TODO (not measured) | TODO (not measured) | TODO (not measured) | TODO (not measured) | pending robot session |

**Sanity checks (reported, not silently fixed):**

1. **Monotonic decrease of |L−R| with gain** — holds: 14.0 → 4.0 → 2.0 counts.
   Each gain increase reduced the residual, consistent with a P controller
   closing in on the disturbance.
2. **Spread vs mean** — the K0 pair (10, 18) has spread 8 counts on a mean of
   14 (57 %); run-to-run variance of the *uncorrected* asymmetry is large.
   K1 and K3 have n=1, so no spread is known for them — the table does not
   pretend otherwise. Consequence for interpretation: the K0→K1 improvement
   (14→4) is far larger than the K0 run-to-run spread (8), so it is a real
   controller effect, not run luck; the K1→K3 step (4→2) is within what a
   single run each can resolve and is treated as indicative, not proven.
3. **Unit conversions** — 1 count = 5.109 mm (K_MM) and = 1.94° heading
   (k/b at b = 151 mm). The heading equivalents above are the *wheel-path
   difference* expressed as the heading change the same arc would produce;
   they quantify the disturbance magnitude, not the robot's actual heading
   drift (that is Table 2's lateral-deviation business).
4. **Measured vs equilibrium model** — at cruise the steady-state difference
   predicted by the linear model is L−R ≈ 9 PWM ÷ (K × 5.109 mm) counts
   (+7 % asymmetry ≈ 9 PWM of disturbance at pwm 130). Model: 1.76 counts at
   K1 (measured 4, residual +2.24) and 0.59 counts at K3 (measured 2, residual
   +1.41); for K0 the model has no finite equilibrium (no correction), so the
   10/18 measured values are the *transient-limited* asymmetry, not a
   steady state. **Hypothesis (labeled as such, not tested):** the measured
   residuals above the cruise-equilibrium model come from (a) the start
   transient — the correction needs travel to build while the asymmetry acts
   from the first PWM step, (b) asymmetry near the dead band / during the
   ramps being larger than the +7 % cruise value, and (c) reduced correction
   authority whenever a wheel sits at the PWM_MIN floor. Distinguishing (a)
   from (b)/(c) would need the telemetry-vs-time logs (Table 3 material) and
   is not claimed here.

**Narrative.** The guide's symptom table maps to this session as follows.
K0 (the required zero-gain reference) shows the "robot always drifts to the
same side" row — with no correction the +7 % right-wheel asymmetry curves
every run the same way, and the end difference is the largest measured
(10–18 counts over 500 mm). K1 and K3 both land in the "good gain" regime:
path visibly straighter, no zigzag, no audible wheel fight. The guide's
"one wheel stutters" row (correction pushing a wheel below the dead band)
cannot occur in this sketch by construction — the per-wheel PWM_MIN floor
(85 on turns / 55 on straights) plus the CORR_MAX = 30 clamp make any
sub-dead-band command impossible — and correspondingly it was never observed;
the expected too-high-gain symptom in this architecture is instead weaving or
clamp bang-bang near the floor, which also was not observed at K3. The choice
**K_SYM = 3.0** rests on: the smallest measured residual (2 counts ≈ 10 mm of
wheel-path difference ≈ 3.9° heading equivalent over 500 mm), zero observed
symptoms, and headroom — at the measured residual the correction demand
(≈ 2 cnt × 5.109 mm × 3.0 PWM/mm ≈ 31 PWM ≈ CORR_MAX) only saturates at the
largest seen disturbances, leaving the clamp as the safety net rather than an
active limiter. K1 remains the documented fallback if a future floor state
(lower battery) makes K3's correction demand grow into the clamp.

**Design note (correction vs dead band).** The guide asks what happens when
the correction would push a wheel below the dead band. This implementation
answers structurally: the profile floor (PWM_MIN) and the correction clamp
(CORR_MAX) are sized so the commanded duty can never go under the dead band —
the worst case is a wheel held *at* the floor, where its speed can no longer
be reduced further and the correction authority on that wheel is momentarily
zero (the trade-off the guide warns about, accepted and documented rather
than hidden). While one wheel sits at the floor the controller still slows
nothing there, but the other wheel's authority is unchanged, so the error
growth is bounded by the same asymmetry that K0 shows — never inverted.

**Fourth gain (checkpoint needs ≥ 4).** TODO (not measured). The table and
this paragraph will take the fourth gain (suggested: 0.3, the halfway point
on the log scale between 0 and 1, or 1.5 to probe between the two working
values) with its runs and symptoms pasted in. Expected from the model: steady
residual ≈ 9 ÷ (0.3 × 5.109) ≈ 5.9 counts — between K0 and K1; its symptom
row should show a partial curve, which would further support the equilibrium
model if it lands near that figure.

**DATA GAPS (need the robot):**
- 4th-gain runs (B500 ×1–2 at K0.3 or K1.5) + one-line symptom → fills the
  last table row and paragraph d
- (optional, strengthens the K0 reference) 1–2 more K0 runs to shrink the
  57 % relative spread before the table is quoted as a mean
- lateral deviation vs gain (K0 vs chosen, 5 runs each) → **Table 2, not
  this table**; symmetry-error-vs-time telemetry → **Table 3**; controller-
  vs-scale error and the 550 mm run-on episode → **Table 4** — deliberately
  excluded here

**Checkpoint item served** ("tuning documented: at least 4 gain values with
the symptoms described"): **PENDING** — 3 real gains with symptoms recorded
(K0, K1, K3); the item flips to SATISFIED once the 4th-gain row above is
filled from a real session. The choice of K3 is already justified and
entered in the sketch (`K_SYM = 3.0f`) independently of that formality.
