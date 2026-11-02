# Calibration-constant audit — 02.11.2026 (desk pass, no robot data)

Method: every constant was read directly from each sketch's constants block
(file:line below) and cross-checked against the reports and `data.txt`.
Nothing was edited as a result of this audit — findings are listed for a
decision, per the task brief.

## 1. Constants — consistent everywhere

### mmPerTick (K_MM) = 5.1087 mm/count — IDENTICAL in all 5 sketches
| file:line | value |
|---|---|
| lab1_stage3_straight/lab1_stage3_straight.ino:28 | 5.1087f |
| lab1_stage4_turns/lab1_stage4_turns.ino:24 | 5.1087f |
| lab1_stage5_protocol/lab1_stage5_protocol.ino:34 | 5.1087f |
| lab1_stage6_autonomous/lab1_stage6_autonomous.ino:37 | 5.1087f |
| lab1_stage7_faultrecovery/lab1_stage7_faultrecovery.ino:41 | 5.1087f |

Provenance: REPORT_STAGE2.md §7.2 (k_mean 5.109, spread 1.54 %, k_geom 5.105,
+0.07 %). data.txt `[k] L_sec 1000, wheel_d 65.0` matches the calibration
input. Consistent. (Cosmetic: greeting strings print `k=5.108`/`k=5.109` —
both are roundings of 5.1087, display only. REPORT_STAGE5 header says
`k = 5.108`, REPORT_STAGE2 says `5.109` — same class, no action needed.)

### trackMm (b_spin) = 151.0 mm — IDENTICAL in the 4 sketches that turn
| file:line | value |
|---|---|
| lab1_stage4_turns/...ino:33 | 151.0f |
| lab1_stage5_protocol/...ino:59 | 151.0f |
| lab1_stage6_autonomous/...ino:62 | 151.0f |
| lab1_stage7_faultrecovery/...ino:66 | 151.0f |

### trackMmPivot (b_pivot) = 145.0 mm — IDENTICAL, PROVISIONAL (as declared)
| file:line | value |
|---|---|
| lab1_stage4_turns/...ino:34 | 145.0f |
| lab1_stage5_protocol/...ino:60 | 145.0f |
| lab1_stage6_autonomous/...ino:63 | 145.0f |
| lab1_stage7_faultrecovery/...ino:67 | 145.0f |

Provenance: [SIM] model fit (REPORT_STAGE4_SIM_PREFILL §A); real 360° pivot
run still owed — that is Block B, and `stage4_accuracy_template.md` §Table 2
has the calibration rows for it. Consistent AND correctly flagged
provisional everywhere it appears.

### pwmCruise = 130 (drive) / 110 (turn) — IDENTICAL
| file:line | value |
|---|---|
| lab1_stage3_straight/...ino:36 | PWM_CRUISE 130 |
| lab1_stage4_turns/...ino:35 | PWM_CRUISE 110 (turns slower) |
| lab1_stage5_protocol/...ino:47,61 | PWM_CRUISE_DRV 130 / _TRN 110 |
| lab1_stage6_autonomous/...ino:50,64 | same |
| lab1_stage7_faultrecovery/...ino:54,68 | same |

### kpSync (K_SYM) = 3.0 PWM/mm — IDENTICAL; CORR_MAX clamps match too
| file:line | value |
|---|---|
| lab1_stage3_straight/...ino:73,76 | K_SYM 3.0f, CORR_MAX 30 |
| lab1_stage4_turns/...ino:63,64 | K_SYM 3.0f, CORR_MAX 20 (turn) |
| lab1_stage5_protocol/...ino:43,52,66 | K_SYM 3.0f, _DRV 30 / _TRN 20 |
| lab1_stage6_autonomous/...ino:46,55,69 | same |
| lab1_stage7_faultrecovery/...ino:50,59,79 | same |

Turn-specific kp: there is exactly ONE symGain shared by both controllers
(stage7:283 `symGain = K_SYM`, runtime-tunable via `K<value>`) — by design,
not a divergence. CORR_MAX 30 straight / 20 turn is consistent across every
sketch that has both.

### CTRL_DT_MS (named CTRL_MS) = 20 ms, 50 Hz — IDENTICAL
| file:line | value |
|---|---|
| lab1_stage3_straight/...ino:48 | 20 |
| lab1_stage4_turns/...ino:47 | 20 |
| lab1_stage5_protocol/...ino:41 | 20 |
| lab1_stage6_autonomous/...ino:44 | 20 |
| lab1_stage7_faultrecovery/...ino:48 | 20 |

### S / N_rev / WHEEL_D = 20 / 40 / 65.0 — IDENTICAL everywhere incl. data.txt
| file:line | value |
|---|---|
| lab1_stage3_straight/...ino:25–27 | SLOTS 20, N_REV 2*SLOTS=40, WHEEL_D 65.0f |
| lab1_stage4_turns/...ino:21–23 | same |
| lab1_stage5_protocol/...ino:31–33 | same |
| lab1_stage6_autonomous/...ino:34–36 | same |
| lab1_stage7_faultrecovery/...ino:38–40 | same |
| data.txt:3 | slots = 20, wheel_d = 65.0 |

Matches REPORT_STAGE2 §7.1 (20 slots visual, 40 counts/rev electrical, both
wheels, CHANGE = both edges).

### Stage-7 fault constants — self-consistent (REPORT_STAGE7 §2 table matches the sketch)
| constant | sketch (lab1_stage7_faultrecovery.ino) | report §2 |
|---|---|---|
| N_STALL_STEPS | :130 = 3 | 3 |
| STALL_WIN_MS | :123 = 80 | 80 |
| ZERO-output abort budget | derived: 3 × 80 = 240 ms (printed in the err line, :505) | "240 ms" |
| STALL_RATIO_PCT | :128 = 20 | 20 |
| SLIP_* / HOLD_* / SPOOL_GRACE_MS / STALL_MOVED_CNT | :134–155 | all match |

Naming note: `ZERO_OUTPUT_ABORT_MS` does not exist as a constant — the abort
budget is computed as N_STALL_STEPS × STALL_WIN_MS. Not an inconsistency; a
naming alias to keep in mind when reading the task list.

## 2. INCONSISTENCY FOUND — dead band / PWM_MIN provenance (FLAGGED, not fixed)

Two different dead-band stories live in the repo:

- **Stage 1 report (REPORT_STAGE1.md §3, checkpoint item):** floor search gave
  "50 moves / 45 does not" → adopted working value **55** with ~10 % margin,
  and §3 says this 55 is "`PWM_MIN` in stage 3".
- **Stages 3–7 sketches (all five):** `PWM_MIN = 85` with the comment
  "floor-measured under-load dead band (~75) + margin"
  (stage3:39, stage4:39, stage5:40, stage6:43, stage7:47).
  REPORT_STAGE3_S5_ASSEMBLED.md:128 also states "PWM_MIN 85; dead band ≈ 75".

So either (a) the dead band was re-measured at ≈75 in a later session and the
stage-1 report was never updated, or (b) 75 is an estimate that was never
actually measured and 55 is the only floor-measured number. Supporting
evidence for a higher real threshold: stage-4 session row 9 recorded 3×
start-stall AT PWM_MIN 85 on a sagged pack — the threshold is pack-dependent
(guide's voltage warning), which explains why 55 worked on 14.10 and 85 was
needed later.

Related headroom arithmetic worth noting: CORR_MAX 30 with the old floor 55
meant corrections could pull a wheel to 25 (below today's floor); with
PWM_MIN 85 the same clamp keeps wheels ≥ 55 — coincidentally exactly the old
working value.

**Decision needed (ASK):** record the 55→85 evolution in REPORT_STAGE1 §3
(and where 75 came from), or add a dead-band re-measurement (wheels on floor,
`+`/`-` search at 5-step resolution) to Block A of the next session. No .ino
value should change without that re-measurement.

## 3. Minor comment-lag (FLAGGED, comment-only, no action taken)

- lab1_stage4_turns.ino:33 comment says track 151.0 "(PROVISIONAL)", but
  session_1440_confirmation.md concluded "151.0 CONFIRMED-with-kick-caveat".
  The stage-5/6/7 copies carry the same comment. Comment-only edit — allowed
  only on your say-so (constraint: no .ino edits this pass).
- REPORT_STAGE4.md:81 still has the blank "calibrated track: ______ mm" line
  (the number lives in session_1440_confirmation.md and the pre-fill §A).
  Filling it is a report edit, offered together with the 55/85 decision.

## 4. Summary

- Zero numeric inconsistencies in: K_MM, b_spin, b_pivot, PWM_CRUISE,
  K_SYM, CORR_MAX, CTRL_MS, SLOTS/N_REV/WHEEL_D, and the whole stage-7
  constant block (sketch ↔ report ↔ data.txt all agree).
- One provenance inconsistency: dead band 55 (stage-1 report) vs ~75
  (sketch comments, PWM_MIN 85) — needs either a doc note or a Block-A
  re-measurement. ASKING.
- Two comment-lag items (PROVISIONAL wording, one blank report line) —
  comment/doc-only, on request.
