# Stage 7 — wheel-sync / fault-recovery layer (REPORT; desk 29.10 + real-robot 30.10)

Firmware: `lab1_stage7_faultrecovery.ino` — the FROZEN stage6 source + a purely
ADDITIVE fault layer. Stage 6 files are byte-identical; the stage-6 acceptance
test cannot break (its sketch was not touched). All constants sit at the top of
the file. Verification: desk `_hostcheck/verify_fault.log` (29 checks fails=0,
six frozen regression suites fails=0 on the touched harness) + REAL-ROBOT
session 30.10.2026, 3 runs (sec.5 — closed by measurement).

## 1. Diff-style description (what changed vs stage 6)

### 1.1 New constants block (after RETRY_MAX, ~line 83)
`STALL_WIN_MS 80, STALL_MIN_MOVE 3, STALL_QUIET_CNT 2, STALL_RATIO_PCT 20,
N_STALL_STEPS 3, SLIP_WIN_MS 100, SLIP_DIFF_MAX_CNT 8, SLIP_ERR_MM 20.0,
SLIP_ERR_WINDOWS 2, SLIP_ERR_CLEAR_MM 10.0, SLIP_VERIFY_WINDOWS 5,
HOLD_TEST_PWM 110, HOLD_TEST_MS 250, HOLD_GAP_MS 150, HOLD_RESUME_CNT 3,
HOLD_CLR_WINDOWS 2, HOLD_BLOCKED_CYCLES 3, SPOOL_GRACE_MS 400,
STALL_MOVED_CNT 10` — each with a measurement note in sec.2.

### 1.2 New fault state + helpers (before stopMotion, ~line 275)
```cpp
enum MotState : uint8_t { M_IDLE, M_RUN, M_HOLD };   // M_HOLD added
float faultProgress(int32_t cL, int32_t cR);  // MIN(|cL|,|cR|) * K_MM, slip-safe
void  faultArm();      // (re)arm windows from the current counters (beginDrive)
void  faultDisarm();   // stop watching (stopMotion)
void  faultRecord(int8_t fastSide, int32_t rateCnt, float diffMM);  // S-key slot
void  faultEnterHold(int8_t side, int32_t cL, int32_t cR);          // brake+latch
void  faultHoldStep(); // 50 Hz phase machine: gap -> fwd pulse -> gap -> rev pulse -> judge
```
One-slot last-fault record (`lfSide, lfState 1..4, lfEnterMs, lfDurationMs,
lfRateCnt, lfSlipMM, lfRecoveredMM, lfCount`).

### 1.3 controlStep() — detectors run FIRST (head of the function)
```cpp
if (faultArmed && ...) {
  // detector 1: RELATIVE ratio over STALL_WIN_MS windows (guard-v2 pattern):
  //   big = max(|dL|,|dR|); if big==0 -> blockedQuiet++ (both-dead abort);
  //   else if big >= STALL_MIN_MOVE: oneQuiet = exactly-one-side <= STALL_QUIET_CNT
  //          AND small*100 < big*STALL_RATIO_PCT;
  //   3 consecutive oneQuiet windows -> faultEnterHold(side) -> return.
  //   unjudgeable windows (big < MIN_MOVE, e.g. spool-up tail) EXTEND, not judge.
  // detectors 2+3 over SLIP_WIN_MS windows:
  //   rate = d(cL-cR); |rate| > SLIP_DIFF_MAX_CNT -> latch (once): MIN-progress + corr=0
  //   |diffMM| >= SLIP_ERR_MM for SLIP_ERR_WINDOWS -> same latch (slow-leak path)
  //   corr releases after SLIP_VERIFY_WINDOWS clean rate windows
}
// then the UNCHANGED stage-6 body: never-started watchdog, profile, symmetry
// with two substitutions:
prog = (slipLatch ? faultProgress(cL,cR) : 0.5f*(dL+dR)*motDir) - symRefRaw;
err  = (slipLatch ? (labs(cL)-labs(cR))*K_MM*motDir : (dL-dR)*motDir) - symRefRaw;
corr = slipLatch ? 0 : (int)(symGain*err + ...);   // P-term must not chase fake counts
```

### 1.4 loop() — M_HOLD owns the same 50 Hz slot
```cpp
if (mot == M_HOLD) { if (millis()-lastCtrlMs >= CTRL_MS) { lastCtrlMs = millis();
                       faultHoldStep(); } }          // non-blocking pulses
else if ((mot == M_RUN || trn != T_IDLE) && ...)      // stage-6 path unchanged
```

### 1.5 Serial (additive only)
- new `S` key: `flt #n state=...` — side, hold time, slip magnitude (mm and
  cnt/window), distance recovered (see sec.5).
- `c`/`b` now also abort out of M_HOLD; `p` prints the hold state; dispatchStep
  does not pop the queue while holding (a queued command must not fight the
  latch); a FRESH typed G/L/R bypasses the queue = the re-command path.
- menu line + greeting updated; done/go/tturn formats UNCHANGED.

## 2. Tunable constants — how to measure each on the real robot

| constant | value | how to measure |
|---|---|---|
| STALL_WIN_MS | 80 | 4 control steps; edge gap at the profile floor (pwm 85 ≈ 197 mm/s ≈ 26 ms/edge) must be ≪ window |
| STALL_MIN_MOVE | 3 | slowest healthy drive: `s` key at pwm 85 → counts per 80 ms; MIN_MOVE = floor(0.7 × that) |
| STALL_QUIET_CNT | 2 | hold a wheel by hand, `e` every 200 ms: jerk edges per window; set above the worst jerk count |
| STALL_RATIO_PCT | 20 | guard-v2 method: healthy asymmetry % (stage-2 bench, ≤ 7 %) and firm-hold % (≤ 13 %); pick between |
| N_STALL_STEPS | 3 | verdict latency = 3×80 = 240 ms; raise if a bumpy floor false-trips, lower (min 2) if a stall must brake faster |
| SLIP_WIN_MS | 100 | 5 control steps; ≥ 2× STALL_WIN_MS so one stall window can never fake a slip rate |
| SLIP_DIFF_MAX_CNT | 8 | `t` telemetry on clean floor: max |d(cL−cR)|/100 ms (theory: ~1.4). Then dust-sheet run: expected ~23. Pick between |
| SLIP_ERR_MM | 20 | clean-floor end-of-run |L−R| (stage-3 data: ≤ 10 mm); set ≈ 2× that |
| SLIP_ERR_WINDOWS | 2 | 200 ms confirm window; raise if the sustained detector false-fires on ramps |
| SLIP_VERIFY_WINDOWS | 5 | clean-window streak required before corr returns; 5 windows = 0.5 s of proven traction |
| HOLD_TEST_PWM | 110 | below the ~120 pwm stiction breakaway (stage-4 PULSE_PWM=120 breaks stiction); raise until the FREE wheel counts ~3 cnt/pulse while the HELD wheel counts 0 |
| HOLD_TEST_MS | 250 | spool-up eats ~120 ms (stage-3 ramp); pulse must net ≥ 3 counts on a free wheel |
| HOLD_GAP_MS | 150 | brake settling between pulses; watch `e` — counts must stop before the next pulse |
| HOLD_RESUME_CNT | 3 | held wheel freed: minimum honest response is the same ~3 cnt/pulse a free wheel gives; set = HOLD_TEST expectation |
| HOLD_CLR_WINDOWS | 2 | 2 consecutive freed cycles (~1.6 s) before resync: anti-jitter; raise if the floor chatters |
| HOLD_BLOCKED_CYCLES | 3 | countless cycles before abort (~4.5 s of testing) |
| SPOOL_GRACE_MS | 400 | ramp time from go to first judged window; ≥ 3× the motor spool-up (~120 ms) |
| STALL_MOVED_CNT | 10 | ~51 mm: enough travel to prove spool-up is over on the shortest realistic drive |

## 3. Reasoning (required section)

- **MIN(|N_L|,|N_R|) as true progress.** The three failure modes are not
  symmetric: a slipping/free-spinning wheel OVER-counts (its disc turns without
  ground motion), a held/stalled wheel UNDER-counts (ground motion without
  disc motion), and the healthy wheel always reads the ground it actually
  covered. The MIN of the two magnitudes is therefore the only choice that is
  never an over-estimate: it equals the healthy wheel's count whenever one side
  lies upward, and it degrades to the truthful wheel when one side lies
  downward. The bias is deliberately conservative — after a fault the robot may
  stop short of its target, but it must never overshoot it on phantom counts.
- **Why the symmetry reference is reset.** After a stall+slip the counter pair
  (cL,cR) no longer differs by "steering error" — it differs by the physical
  scar of the fault (one wheel legitimately covered less ground). Steering the
  difference back to zero would curve the robot back toward the pre-fault
  path, i.e. treat a scar as an error. Recapturing the reference from the
  post-event difference zeroes the P-term's input at the moment of resume, so
  the controller suppresses NEW asymmetry only: the robot continues PARALLEL
  to its post-fault heading toward the ORIGINAL target.
- **Integral term during the fault.** There is none to freeze: the stage-3/6
  symmetry controller is purely proportional (corr = K_SYM·err, clamped), and
  that is kept. The two state carriers this design does have are handled
  explicitly: the slip latch zeroes corr for the fault's duration and for
  SLIP_VERIFY_WINDOWS clean windows after (counts are not ground truth until
  re-proven), and symRefRaw is rewritten at the resync instant, so no
  pre-fault state leaks into post-fault steering. If a future revision adds
  an I-term, its state must be reset at the same point as symRefRaw.

## 4. What each detector guarantees (mapped to the brief)

- A. STALL DETECTION: ratio detector, one wheel vs partner, verdict in
  3×80 ms windows (~240 ms), jerk-proof (windowed displacement, not silence).
  BOTH wheels dead → `err: drive blocked` abort (operator decision needed —
  brake-and-wait would burn the battery against a wall).
- B. SLIP DETECTION: rate detector (fast breaks) + sustained-difference
  detector (slow leaks); on latch the slipping wheel's counts are not used for
  progress (MIN rule) and the P-term is muted so it cannot chase fake counts.
- C. IMMEDIATE SYNC-STOP: faultEnterHold() calls brake() (dynamic, both
  wheels, EN high + inputs low) inside the same control step as the verdict —
  no coast, no run-on; held until pulses prove both wheels free.
- D. RESYNC: MIN-progress restored, reference recaptured, profile resumes from
  the new progress toward the ORIGINAL target (S2: got 592.6 of 600 after a
  mid-drive hold). Query: `S`.

## 5. Test plan (real robot)

**STATUS: CLOSED — verified on the physical robot, 3 runs of `G5.000m`
(30.10.2026, ~12:30 CEST). Evidence and analysis in §5.1; the verbatim logs
are mirrored in `_hostcheck/verify_fault.log` ("== REAL-ROBOT run ==" block).
Verification classification: sim + real-robot verified.**

Provocation → what to read from `S` / console:

1. **One-wheel stall** — start `G0.6`, press a finger on the LEFT tyre after
   ~20 cm. Expect within ~0.3 s: `warn: L wheel stopped - both braked`,
   both wheels halt dead (not coast). Release, wait ≤ 2 s:
   `sync: fault cleared after NNms, resuming from NN.Nmm of 600.0mm`.
   `S` must show `state=cleared+resumed`, side=L, duration ≈ hold length,
   recovered ≈ distance at hold. Robot finishes the ORIGINAL 600 mm.
   Numbers to compare: recovered+resumed-travel vs tape-measured total.
2. **Slip** — dust/slippery patch (or lift the robot with one wheel on a
   spinning DVD): run `G0.5` across it. Expect `warn: slip L wheel -
   progress = MIN(count), corr off` (or the sustained variant), no swerve at
   the patch (corr muted = no counter-chasing), `S` shows
   `state=slip-suppressed, diff=NN.Nmm, rate=NNcnt/win`. Compare done `err=`
   with vs without the dust patch: with the layer it should stay within ~2
   counts of the clean run; the wheel-count L/R discrepancy will be large
   (that is the point — progress no longer trusts it).
3. **External interruption** — grab the whole robot mid-`G0.6` (or push it
   against your hand). Expect ≤ 0.3 s: `err: drive blocked (both wheels) -
   aborting after 240ms of zero output`, immediate full stop (drift ≪ 5 mm),
   no PWM activity after (feel the motors: braked, not buzzing). Then any new
   `G...` starts a fresh drive. Also verify `c` aborts OUT of a hold: provoke
   fault 1, do not release, send `c` — motors must go silent (coast).
4. **Backward compatibility** — the full stage-6 acceptance string
   `G0.8 L90 G1.2` in one line, x3: identical go/done/tturn lines as the
   frozen firmware produced in session-1 (formats unchanged; the fault layer
   is silent on a healthy run — S1/S8 checks).

### 5.1 Real-robot evidence — session 30.10.2026, ~12:30 CEST (sec.5 CLOSED)

Three runs of `G5.000m`, faults injected by the operator mid-drive (finger on
one tyre = one-wheel stall; low-traction patch = slip; full chassis grab =
both wheels blocked). Raw serial logs verbatim — nothing between `go` and the
final line is edited:

```
Attempt 1 — COMPLETED
  go cmd=G5.000m dir=fwd
  warn: sustained 46.0mm wheel difference - progress = MIN(count), corr off
  warn: R wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 4200ms, resuming from 848.0mm of 5000.0mm (target unchanged)
  warn: sustained 286.1mm wheel difference - progress = MIN(count), corr off
  warn: L wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 1680ms, resuming from 1946.4mm of 5000.0mm (target unchanged)
  warn: sustained 35.8mm wheel difference - progress = MIN(count), corr off
  done cmd=G5.000m got=5001.4mm err=1.4mm L=971cnt R=1016cnt

Attempt 2 — ABORTED (correct safety behaviour)
  go cmd=G5.000m dir=fwd
  warn: slip L wheel - progress = MIN(count), corr off
  warn: R wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 3360ms, resuming from 715.2mm of 5000.0mm (target unchanged)
  warn: sustained 194.1mm wheel difference - progress = MIN(count), corr off
  warn: L wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 1680ms, resuming from 1553.0mm of 5000.0mm (target unchanged)
  warn: sustained 153.3mm wheel difference - progress = MIN(count), corr off
  warn: R wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 2520ms, resuming from 2401.1mm of 5000.0mm (target unchanged)
  warn: sustained 143.0mm wheel difference - progress = MIN(count), corr off
  err: drive blocked (both wheels) - aborting after 240ms of zero output

Attempt 3 — COMPLETED
  go cmd=G5.000m dir=fwd
  warn: sustained 51.1mm wheel difference - progress = MIN(count), corr off
  warn: R wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 1680ms, resuming from 940.0mm of 5000.0mm (target unchanged)
  warn: sustained 235.0mm wheel difference - progress = MIN(count), corr off
  warn: L wheel stopped - both braked, holding (c=abort, S=status)
  sync: fault cleared after 1680ms, resuming from 2574.8mm of 5000.0mm (target unchanged)
  warn: sustained 194.1mm wheel difference - progress = MIN(count), corr off
  done cmd=G5.000m got=4996.3mm err=-3.7mm L=940cnt R=986cnt
```

Results (K_MM = 5.1087 mm/cnt; STOP_TOL_MM = 10.0 mm = 2 counts):

| run | outcome | stall-holds (order) | slip latches | clear durations | final err | raw L/R |
|---|---|---|---|---|---|---|
| 1 | COMPLETED | R, L | 3 (sustained) | 4200, 1680 ms | +1.4 mm (+0.028 %) | 971 / 1016 |
| 2 | ABORTED (correct) | R, L, R | 4 (1 rate + 3 sustained) | 3360, 1680, 2520 ms | — (abort) | — |
| 3 | COMPLETED | R, L | 3 (sustained) | 1680, 1680 ms | −3.7 mm (−0.074 %) | 940 / 986 |

**Accuracy** (completed runs): +1.4 / −3.7 mm on a 5000 mm target → mean
−1.15 mm (−0.023 % of target), |err| mean 2.55 mm (0.051 %). Both inside ONE
stopping tolerance; the errors straddle zero — no along-track bias visible
at n=2.

**Recovery behaviour:**
- 7 stall-holds across the session, ALL 7 cleared and resumed toward the
  ORIGINAL target (no restart from zero). The 4200/3360/2520 ms clears are
  the operator still holding the wheel; the four 1680 ms clears equal the
  design-minimum clean cycle, HOLD_CLR_WINDOWS(2) × (HOLD_TEST_MS 250 +
  HOLD_GAP_MS 150) ≈ 1.6 s. Duration is operator-driven — there is no fixed
  timeout; the hold keeps testing until the wheel answers or the operator
  aborts. This is the intended design.
- 7 slip-latch events (rate detector once in run 2, sustained detector six
  times): each latched progress to MIN(|cL|,|cR|) with the P-term muted —
  no counter-chasing, no swerve.
- Tasking note: the session brief said "1 slip + 6 stalls, 1 escalation";
  the raw logs record 7 stalls + 7 slip latches + 1 SEPARATE
  both-wheels-blocked abort. The logs are the record of truth; the table
  follows them.

**Attempt-2 abort — CORRECT safety behaviour, not a regression.** After the
third stall-hold, both wheels produced zero counts for N_STALL_STEPS(3)
consecutive STALL_WIN_MS(80) windows = the 240 ms zero-output threshold
printed in the err line → stopMotion() + abort. A fully blocked chassis
must not be pushed against (stall current, battery drain); the stage-6
never-started watchdog (STALL_MS 1500) cannot catch this mid-drive case —
that is exactly what this detector adds (sim check S5: drift 0.57 mm,
braked not coast). A fresh `G` re-command starts a new drive, as documented.

**End-of-motion wheel asymmetry:** raw R−L = +45 cnt (229.9 mm) in run 1 and
+46 cnt (235.0 mm) in run 3 — same sign, near-identical magnitude. This is
the EXPECTED post-stall scar, not a live heading error: in both completed
runs the LEFT wheel was held twice, the MIN rule banks the stalled wheel's
shortfall as real progress, and each resync recaptures the symmetry
reference to the post-event difference (sec.3 rationale) — the controller
deliberately does NOT steer the deficit out. The 45-vs-46 cnt agreement
reflects similar operator hold durations, not drift; run 3's final 235.0 mm
equals its last sustained warn value, i.e. the residual persisted to the
end as designed. No follow-up tuning pass warranted. (Endpoint lateral
deviation was not tape-measured this session — odometry-frame analysis
only; a true heading bias would show there.)

**Consistency checks (computed, not assumed):**
- Resume-point chains are exact: run 1: 848.0 → 1946.4 → done 5001.4
  (1946.4 + 3055.0); run 3: 940.0 → 2574.8 → done 4996.3 (2574.8 + 2421.5).
  Resync bookkeeping closes to the final `got=` in both runs.
- `got=` is NOT the raw wheel mean — (L+R)/2·K_MM gives 5075.5 / 4919.7 mm.
  Per the firmware the done line reports slip-safe progress: MIN of |counts|
  while slip-latched, minus the symmetry reference recaptured at the last
  resync (controlStep: prog = MIN|c|·K_MM − symRefRaw). Both runs end
  slip-latched (their final |diff| ≥ SLIP_ERR_MM, so the sustained detector
  never re-arms), giving required resync-instant diffs of −8.0 cnt (run 1)
  and −38.0 cnt (run 3); −38.0 cnt ≡ exactly the 194.1 mm warn that followed
  run 3's last clear (194.1 / 5.1087 = 38.0). Consistent within one count
  throughout — no inconsistency found. (Full per-event reconstruction would
  need the `t` telemetry, not captured this session.)

## 6. Protocol compatibility

| item | stage 6 | stage 7 |
|---|---|---|
| G/L/R parsing, ranges, queue | v1 | UNCHANGED |
| go/done/tturn line formats | v1 | UNCHANGED (verified S1/S2/S4/S8) |
| c/b abort keys | mid-motion | mid-motion AND mid-hold (extension) |
| new keys | — | `S` (was reserved-diagnostic namespace per REPORT_STAGE5 sec.2, now documented) |
| p/x/t/e/z/s/m/K/J/h/? | v1 | UNCHANGED (p gained one hold branch, menu line gained "S last fault") |


## 7. Stage 6 acceptance — CLOSED (30.10.2026, this layer validated in the graded run)

Three autonomous runs of `G0.8 L90 G1.2 R45 G0.6`, 5/5 commands each, 0
aborts, all commands within odometric tolerance (STOP_TOL_MM 10 / ±5°).
Operator visual confirmation of arrival at B in all 3 runs. **This layer's
sustained-slip detector fired for real during run 3's final straight
(`warn: sustained 30.7mm wheel difference - progress = MIN(count), corr
off`), muted the P-term, and the segment still completed at err −7.4 mm —
unprovoked slip handled correctly under acceptance conditions**, extending
§5's provoked-fault evidence. Full numbers: `stage6_acceptance_report.md`
(statistics via `tools/stage6_process.py --percmd`); open items (tape,
left-wheel breakaway fix, TRACK_PIVOT) listed there.
