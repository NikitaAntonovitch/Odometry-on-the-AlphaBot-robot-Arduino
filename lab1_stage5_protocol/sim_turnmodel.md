# Stage 4/5 — Monte-Carlo simulation of the turn machine (desk, 21.10)

Model calibrated on session-1 evidence, then used to predict the patch effect.
All numbers desk-generated ([SIM] discipline: conclusions phrased "WOULD CLOSE
if real data confirms").

## Model (stochastic, per spin turn)
- stop trigger by construction: diff ≥ 2×(tgt−0.7)° in counts (45° → 22.9, 90° → 46.1)
- healthy turn: both wheels track, per-wheel split ±1, quantisation 1 pair;
  kick folds 2 counts (p=.8) else 1, off one wheel
- collapsed turn (per-START random wheel, p = stall_p): the stuck wheel
  contributes 0–2 counts; the live wheel over-runs to the trigger; after the
  stop, the corrective pulse + coast folds strip D = 8…18 differential counts
  (derived from the observed collapsed rows: 31.0 = diff 16, 25.2 = 13, 13.6 = 7
  vs trigger 22.9)
- patch: per-wheel breakaway rescues a stalled start (residual deep-stall 3 %)

## Calibration (L45, 2000 runs per cell)
| stall_p | mean got | σ | matches session? |
|---|---|---|---|
| 0.20 | 37.1 | 8.8 | no |
| **0.30** | **34.4** | **10.6** | **YES — session: 34.1 / 10.3** |
| 0.40 | 32.0 | 11.5 | σ too wide |
| 0.50 | 29.8 | 11.7 | no |

→ The session's stall rate back-solves to **p ≈ 0.30 per turn** (6/20 observed
collapse rows — consistent).

## Patch prediction (L45, 2000 runs)
| configuration | mean got | mean E | σ | worst | 5 %-rule (|E|≤2.25) |
|---|---|---|---|---|---|
| pre-patch (stall 30 %) | 34.7 | −10.3 | 10.7 | 9.7 | **FAIL** |
| pre-patch bad day (45 %) | 31.2 | −13.8 | 11.6 | 9.7 | FAIL |
| pre-patch good day (10 %) | 39.2 | −5.8 | 6.4 | 9.7 | FAIL |
| **PATCHED (residual 3 %)** | **40.9** | **−4.1** | **2.0** | 9.7 | **FAIL (mean) but σ 2.0** |

- Patch removes ~2/3 of the mean error and ~80 % of the spread.
- The residual −4.1 mean E is the KICK FOLD itself (2 cnt ≈ 7.8° per stop,
  p .8×2+.2×1): after the breakaway patch, the next lever is KICK_PWM 95→80 /
  KICK_MS 60→40 (fold → ~1 cnt ≈ −2..−4° E), predicted mean E ≈ −1…−2 →
  **5 %-rule PASSES** (WOULD CLOSE if real data confirms).
- Patched L90: E −4.6, σ 2.6 → same one-lever story.
- 1440 after patch: healthy 739 cnt = 1432.5 with the kick removed by the
  clean stop → predicted **E ≈ 0…−2** (observed pre-patch −7.5 includes the
  fold); gate-2 re-run predicted PASS with margin (σ ≤ 1 achievable).

## Also note (sim honesty)
The first three sim attempts had sign/model bugs (collapse modelled as
pre-stop under-count; the real mechanism is post-stop fold stripping) — the
evidence (final counts BELOW the by-construction stop trigger) is what pinned
the mechanism. The calibrated model reproduces the session's L45 mean/σ to
0.3/0.3 — good enough for the patch decision, not a substitute for the
re-run.

## Decision
1. Apply the per-wheel breakaway patch (T_PULSE AND-condition) — biggest win:
   σ 10.7 → 2.0.
2. Apply the soft-kick (KICK_PWM 80 / KICK_MS 40) in the same flash —
   removes most of the residual −4° fold.
3. Re-run on robot: 45°×5 + 90°×5 both dirs (gate-2 proxy), 1440°×1L+1R
   (expect |E| ≤ 2 → [prov] retired), pivot 360° L+R (gate 3, 145 un-[SIM]).
