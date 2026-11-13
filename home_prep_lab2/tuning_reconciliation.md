# Tuning & Session Reconciliation — LAB2 (2026-11-13)

Purpose: one source of truth separating **[SIM]** (simulator), **[DEFAULTS]**
(code, host-verified), and **[HW]** (real robot observations). No combination is
called final; nothing succeeded unless evidence says so.

## 1. Where each gain statement comes from

| Value set | Source | Status |
|---|---|---|
| kp=0.3 ki=0 kd=1 b=90 smooth=off | `school_starting_point.md` (sim sweep, 2026-11-06) | [SIM] recommendation only |
| kp=0.25 ki=0 kd=0.3 b=90 smooth=on | `lab2_stage6_track.ino` initializers (2026-11-09) | [DEFAULTS] compile-verified, boot print [HW-PENDING] |
| kp=0.3/1.0 zigzag … kp=0.25/0.8 curve loss | school session 2026-11-09 | [HW] observed |
| "nothing oscillates up to kp=2.0, kd=5 noise limit" | simulator Phase A/B | [SIM] — DID NOT transfer to hardware |

**Bottom line: there is NO final gain set.** The [SIM] predictions were
qualitatively wrong for the real plant (sim never oscillated; hardware zigzagged
at kp=0.3 kd=1.0). The sketch defaults are simply the newest [HW]-tested pair
(0.25/0.3) plus smoothing on (helped straights, [HW]); they are a starting point,
not a result. `ki=0` remains consistent: sim showed no benefit [SIM], and no [HW]
evidence contradicts it.

## 2. The g-command deviation (needs instructor confirmation)

- Written spec (assignment flow + our own D1 checklist): dead-man's switch —
  `g` arms, `y` confirms within 2 s.
- Current code: `g` prints "STARTING IN 3 s", waits 3 s (`millis()`-based,
  no delay), `s` cancels, then starts driving. State machine reset on start.
- This is a **deviation from the written requirement** — rationale: the y-path
  produced repeated [HW] aborts from Serial Monitor line endings (CR/LF eaten as
  wrong answers) and costs a round-trip at the bench. The countdown keeps a
  cancel window and is arguably as safe (robot still held by motorsStop during
  the window).
- **[HW-PENDING instructor OK]** — if the instructor requires the literal g+y
  handshake, revert to the confirm version (git history / `_hostcheck/verify_g_confirm.log`
  has the exact block).

## 3. Hardware observations to date (all [HW], school 2026-11-09)

- Polarity white_line=0 (line ~170–350 low, background ~700–900 high)
- PWM_MIN=85 in code; base<90 triggers GUARD (90 = lowest driving speed)
- Calibration EEPROM-saved; sensor 0 weakest range
- Sign of correction correct
- Loop 20 ms, over=0 (trace off)
- Tuning: 0.3/1.0 → fast zigzag (DRY u saturating ±255); 0.2/0.3 off → zigzag;
  0.2/0.3 on → straights OK, curves lost; 0.25/0.3 on → curve lost;
  0.25/0.8 on → curve lost (initial left spin, ignored line). kp=0.12 runs invalid.
- Boot CONFIG anomaly (kp=0.000/kd=1.000/base=0) = stale-hex suspicion [UNVERIFIED cause]

## 4. Code state vs docs (as of 2026-11-13)

- Bug 1 fix NOW APPLIED (host tests 9/9, `_hostcheck/verify_bug1_fix.log`) —
  docs that said "not applied" are superseded by this file.
- `next_session_plan.md` §1 step 1–3 (send fix prompts) is DONE — do not re-run.
- Still stale after this reconciliation: `school_starting_point.md` §0/§1 (see §1
  above), `stage3_start.md` pre-flight (C expectations + g+y procedure),
  `next_session_plan.md` step 7 "send g y" and step 8 kp candidates.
  These remain unedited on purpose until the instructor rules on the countdown;
  this file is the authoritative overlay.
