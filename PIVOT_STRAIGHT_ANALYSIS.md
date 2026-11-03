# Session 30.10.2026 — pivot stall, straight-line oscillation growth (desk analysis)

Status: ANALYSIS ONLY — no firmware modified. All quotes from
`lab1_stage7_faultrecovery/lab1_stage7_faultrecovery.ino` at HEAD 17f1e55.
Operator-confirmed facts: same session as the L45 isolation battery;
`z` WAS pressed between the three identical L90 pivotL runs; one more
diagnostic (spin L90 ×3) approved.

## A. L135 stall-without-abort (3 identical tel lines at prog=68.6)

- **H1 ("below threshold, observed not acted"): CONFIRMED as the mechanism.**
  The turn controller's only mid-motion progress guard is the STALL_MS
  watchdog (:825: `else if (millis() - lastProgressMs > STALL_MS)`), with
  `STALL_MS = 1500` (:49). `lastProgressMs` refreshes on ANY count from
  EITHER wheel (:824: `if (cL != 0 || cR != 0) lastProgressMs = millis();`).
  In pivotL the planted left wheel produces ~0 counts by design, so progress
  detection rests on the right wheel alone. The duplicate telemetry lines
  span ~2–3 × 100 ms tel period (~300 ms stuck) — far below 1500 ms — so the
  watchdog correctly did NOT fire. The stage-7 windowed ratio detector
  (N_STALL_STEPS × STALL_WIN_MS ≈ 240 ms verdict) would have caught exactly
  this, but it lives in `controlStep()` (straight drives only, :486/:377);
  `turnStep()` never runs it. So: seen, below threshold, by design gap.
- **H2 (spin symmetry controller mis-applied in pivot): REFUTED.** The
  pivot branches explicitly bypass the symmetry controller:
  `} else if (tmode == M_PIVOT_L) { motorL(0); motorR(tsign > 0 ? +pwm : -pwm); }`
  (turnStep, after the M_SPIN branch). No corr term executes in pivot mode.
- **H3 (floor/PWM interaction at the dead-band edge): PARTIAL/UNCLEAR —
  consistent but unproven.** pwm=110 (PWM_CRUISE_TRN) is comfortably above
  PWM_MIN 85, but a transient floor irregularity or momentary stiction under
  the single driven wheel can stall it for ~300 ms. The tel pwm=110 constant
  across the stall shows the controller did not react (it cannot — no
  detector). Cannot distinguish floor bump vs micro-stiction from telemetry
  alone; the ~1 s of missing progress (68.6 → 72.7 in one step) suggests
  stall + catch-up rather than a clean 300 ms blip.
- Extra anomaly (b): L=3 in pivotL — the planted wheel slipped ~15 mm. See B.
- Extra anomaly (c): under-rotation −3.8° — see D.

## B. L=3 slip on the planted wheel: MECHANICAL (firmware does zero the PWM)

`turnStep()` pivot branch: `motorL(0);` — explicit zero, not floating. The
3 counts on the planted wheel are therefore mechanical: tyre creep under the
turning torque, gearbox backlash vibrating the disc, or disc rub — exactly
the guide's "counts with no distance travelled" warning. Note the firmware
counts them honestly into got= (progressDeg() uses the moving wheel in
pivot? — verify: progressDeg uses (cR−cL) semantics per mode; the 3 planted
ticks DID enter the 65/3 accounting and got=131.2 = 66 pairs × 2.0175 −
actually 33 pairs × 2.0175 × 2 ... consistent to within the printed
precision). The MIN-progress rule of stage 7 does NOT apply here (fault
layer disarmed in turns), so nothing masked it either.

## C. Straight-line oscillation amplitude tripled (±5 → ±15.3 mm)

- **H4 (retained state): REFUTED.** `beginDrive()` → `faultArm()` (:345–362)
  resets EVERY controller-state carrier per motion: `slipLatch = false`,
  `symRefRaw = 0.0f` (:353–354), window counters zeroed (:351–352, 355–359).
  The symmetry controller is purely proportional (no I-term exists to
  retain — REPORT_STAGE7 §3). Nothing persists between runs.
- **H1 (battery sag) vs H2/H3 (surface/placement): H1 + H3 jointly most
  likely.** Evidence from the log itself: run 2's FIRST sample already shows
  pwmL=85/pwmR=102 (corr +17) — the controller saw a real heading error at
  step 1, i.e. the robot did not start straight (H3). The oscillation
  amplitude of a P controller scales with the initial error it is fighting;
  a skewed start on a dirtier patch (H2) explains the ±15.3 vs ±10.2.
  Battery sag (H1) raises the effective gain in mm-per-pwm terms only
  slightly and cannot create the first-sample asymmetry by itself.
- **Cheapest disambiguation: start-position discipline.** Re-run G0.5 twice
  with (i) the wheel axis on a tape cross and (ii) voltage noted before each.
  If amplitude follows the placement care and not the voltage, H3 wins.
  (One 60-second test, no reflash.)

## D. Pivot under-rotation: NOT a pure TRACK_PIVOT error (documented)

- L90: moved 41 ticks vs nominal 44.6 at b=145 → deficit 3.6 ticks (7.2°).
- L135: moved 65 ticks vs nominal 66.9 → deficit 1.9 ticks (3.8°).
- Single-value refits disagree: L90 → b_pivot 157.6 mm; L135 → 149.2 mm.
- Two-point fit (got = tgt·(145/b) − off): b = 134.8 mm, off = 14.0°.
  But that fit predicts L90 rep 4 (84.8) as 82.8 — a 1-count miss — and a
  constant 14° offset is physically implausible as a scale error.
- Mechanistic read: the stop-early lead (STOP_TOL_DEG 0.7 + TURN_STOP_LEAD_DEG
  7.8 = 8.5°, .ino:846) was calibrated on SPIN brake dynamics. In pivot the
  single driven wheel brakes with no counter-torque; the residual actually
  delivered after the stop is 1.3° (L90) and 4.7° (L135) instead of the
  assumed +8.5°. The deficit in ticks (3.6 vs 1.9) is neither constant nor
  proportional → a compound of lead mis-calibration for pivot mode + stall
  event in L135 + quantisation (1 pivot count = 2.02° at b=145).
- **Conclusion: do NOT re-fit TRACK_PIVOT from this data** (the task's own
  warning, confirmed numerically). The honest fix path is the planned 360°
  multi-rev pivot calibration on a clean floor, plus a pivot-specific lead
  measurement (the Block-C recipe run in pivot mode).

## Recommendation: (c) PROCEED with one diagnostic — spin L90 ×3 — pivot deferred

The Stage-6 acceptance route (`G0.8 L90 G1.2 R45 G0.6`) uses SPIN turns;
every observed straight-line failure mode so far still finished in
tolerance (today: done err −7.0 mm on ±10 tol, L/R 96/97). Pivot issues do
not block the route. But R45 remains the known weak point (limitation f),
so the spin L90 ×3 with telemetry answers the only question that matters
before acceptance: is TODAY's spin path still the 89.2°/±9%-asym path of
the isolation battery, or has the session's drift reached the spin
controller too.

**Exact next command sequence (user approved one diagnostic):**
1. `z` → `t` (telemetry on)
2. `L90` → capture go/tel/tturn verbatim
3. `z` → `L90` again, `z` → `L90` a third time
4. Read: got ≈ 89.2 ± 1 pair AND |asym| ≤ 2 in all three → proceed to
   Stage-6 acceptance (Phase 2 ×3). Any spin L90 ≤ 87 or asym ≥ 4 →
   STOP, session over, offline analysis with the captured streams.

If proceeding after clean spin L90s: acceptance route skips pivot entirely;
pivot under-rotation + slip + no-stall-detection documented as future work
(analysis doc + REPORT_STAGE4 §pivot note).
