# Protocol-decision verification (task item 7) — 02.11.2026

Scope: verify that every protocol decision the guide requires at stage 5
(checkpoint 5: "the rule for negative values and the units decided and
recorded") is explicitly recorded in the repo's protocol documentation.
Sources checked: `lab1_stage5_protocol/REPORT_STAGE5.md` §1 (the decisions
section), the frozen firmware comment block
(`lab1_stage5_protocol.ino` header / `lab1_stage7_faultrecovery.ino:157–167`),
and the robot-verification rows in REPORT_STAGE5 §4.

Verdict: **all five decisions are recorded AND robot-verified. No proposal
paragraph needed; nothing added by this audit.**

## 1. G command unit and conversion to odometry unit

RECORDED — REPORT_STAGE5.md §1:

> "**Units:** G takes SIGNED metres (negative = backwards) per the guide's
> table; converted to odometry mm at the exec layer."

Firmware mirror (stage-7 sketch, :158–160, same text in the frozen stage-5
source): "command string: G<metres> L<deg> R<deg> ... G: SIGNED metres ...
converted to mm here." Range decision sits beside it: G 0.005…5.000 m
(stage-3 envelope).

## 2. Negative G meaning (backwards)

RECORDED — same line as above: "G takes SIGNED metres (negative = backwards)".
Robot-verified — REPORT_STAGE5.md §1, §2:

> "Robot-verified: `G-0.2` → dir=back; `G-0.300m got=-293.8 err=+6.2` in-window."
> (§2 table rows G−0.2 / G−0.3: both completed, err ≤ 6.2 mm)

## 3. Negative L/R handling

RECORDED — REPORT_STAGE5.md §1:

> "**Sign rule:** direction for L/R lives in the LETTER; a negative number is
> taken as its absolute value (one convention across stages 3/4/5)."

Firmware mirror (stage-7 sketch :162–163): "L/R: direction lives in the
LETTER; a negative number is taken as its absolute value (same rule as
stage 3 F/B and stage 4 L/R — one convention everywhere)."

## 4. Out-of-range argument behaviour

RECORDED — REPORT_STAGE5.md §1:

> "**Ranges:** G 0.005…5.000 m (stage-3 envelope), L/R 5…1440° (stage-4
> envelope, multi-rev calibration needs 1440). Out of range → ONE err line,
> no motion, the rest of the string still executes."

Robot-verified — REPORT_STAGE5.md §4 (session-1 range block, commit fe8a7ee):
5 err lines (`G99`, `L3`, `L2000`, `G0.001`), parser survived, the next valid
command ran. Checkpoint-5 item checked [x] on the same evidence.

## 5. Case sensitivity (G vs g)

RECORDED — REPORT_STAGE5.md §1:

> "**Command format:** one line, commands `G<metres> L<deg> R<deg>` separated
> by any non-alphanumeric characters; letters case-insensitive."

Robot-verified — REPORT_STAGE5.md §4 row: "lowercase `g0.1 l45`: ran
(case-insensitivity live)". The guide's `case 'G': case 'g':` suggestion is
thereby covered by a case-folding design rather than doubled case labels;
the recorded outcome is equivalent.

## 6. Adjacent decision the guide also asks for (garbage tolerance) — recorded

> "**Garbage tolerance:** unrecognised characters skipped with no side effect.
> Robot-verified: `Xyz ,.! Q7 G0.1` → only the G ran, zero err lines."
(REPORT_STAGE5.md §1; one reserved-character trade-off documented: the single
diag keys c b e m p s t x z h K J S remain live inside junk strings.)

## Conclusion for the checklist

Checkpoint-5 "rule for negative values and the units decided and recorded":
SATISFIED with quotes above — no new paragraph proposed, no doc edited.
