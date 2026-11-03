# Non-determinism on L90 spin — root-cause analysis (session 30.10.2026, same power-on)

Status: DESK ANALYSIS ONLY — firmware untouched. All quotes from
`lab1_stage7_faultrecovery/lab1_stage7_faultrecovery.ino` at HEAD 77e941c.
Operator-confirmed: `z` before every run; same power-on as the pivotL block
(no reset); all four `go` lines printed `mode=spin`.

## The evidence, made consistent first

Every run is arithmetically self-consistent with got = (|N_L|+|N_R|)/2 ×
3.877°/pair — including runs 1–2, where the left wheel contributed (almost)
nothing:

| run | cmd | got | N_L | N_R | pairs×3.877° | asym |
|---|---|---|---|---|---|---|
| 1 | L90 spin | 85.3 | **0** | 44 | 85.3 ✓ | −44 |
| 2 | L90 spin | 87.2 | **−2** | 43 | 87.2 ✓ | −45 |
| 3 | R90 spin | −87.2 | 28 | −17 | 87.2 ✓ | +45 |
| 4 | L90 spin | 89.2 | −22 | 24 | 89.2 ✓ | −2 |

The controller's progress signal is the MEAN of the two wheel magnitudes,
so with L dead the turn simply needs ~2× the ticks on R before the
stop-early condition fires (R ≈ 42–44 = exactly what runs 1–2 show). These
runs are not corrupt logs — they are genuine spin-mode runs in which the
left wheel never (or barely) rotated while being commanded.

## H1 — mode-state leak (tmode not reset): REFUTED

`tmode` is written in exactly three places: initialisation
(:638 `static TurnMode tmode = M_SPIN;`), the `m` diagnostic key
(:1018 cycles spin→pivotL→pivotR), and `turnStart` itself
(:715 `tmode = (TurnMode)mode;`). Every L/R command dispatches through
`execTurn`, which passes the CURRENT global tmode AND PRINTS it:

> :923 `turnStart(deg, tmode, letter == 'L' ? +1 : -1);`
> :927–928 `Serial.print(F("deg mode=")); Serial.println(tmode == M_SPIN ? F("spin") : ...);`

The go line is therefore a live read-out of the very variable H1 claims
leaks. All four runs printed `mode=spin` — so tmode WAS M_SPIN at dispatch,
after the pivot block, without any reset. Additionally `z` only touches
encoders (`zeroEncoders()`, :244: counters + snapshots, nothing else) —
it never needed to reset tmode, because tmode was never wrong. H1 is
refuted by the log itself; the "leak" is an illusion produced by a
spin run whose left wheel is physically stalled (a spin with one dead
wheel is kinematically indistinguishable from a pivot about that wheel).

## H2 — a persistent "previous direction" variable: REFUTED

Source scan for any static last/prev/prior state: `lastCtrlMs`,
`lastProgressMs`, `lastTelMs`, `coastLastMs`, `lastAchievedMs`,
`lastCharMs` (all timestamps), `prevL/prevR` (:1183, speed-window scratch,
overwritten every 500 ms), `coastSnapL/R`, `stopSnapL/R` (reset in
`stopTurn()` :747–756 and `turnStart()` :718). None feeds the wheel-drive
decision. The only directional state is `tsign`, written fresh by
`turnStart` (:716) from the command letter every time.

## H3 — parser defaults to the last used mode: REFUTED

The parser has no mode token at all: `case 'L': case 'l': case 'R':`
(:1010–1011) only buffers the letter and number (:1014–1015); dispatch is
`execTurn(letter, arg)` → `turnStart(deg, tmode, …)` (:923). There is no
code path in which a bare `L90` inherits `pivotL` — and again, the go line
would have printed `pivotL` if the global tmode had still been pivotL.
The `m` key (:1017–1021) is the only mode selector, and its state is
printed on every dispatch.

## H4 — `z` (odoReset) corrupts mode: REFUTED

`zeroEncoders()` (:244) touches only `cntL/cntR/snapL/snapR` under
interrupt-off. It neither reads nor writes `tmode`. `handleChar`'s `z`
case (:1055–1060) additionally stops any running motion first. No mode
interaction.

## What actually happened — CONFIRMED (high confidence): left-wheel breakaway failure inside spin mode

The decisive fact is run 3: R90 spin drove the LEFT wheel 28 ticks — the
left motor, H-bridge channel, encoder and wiring all work. Runs 1–2 show
that same motor energised (the symmetry controller had corr clamped at
−20, so `magL = pwm + 20` ≈ 105–130 through the whole turn) yet producing
0/−2 ticks. A motor commanded ≥105 pwm that does not move while its
partner does is a MECHANICAL breakaway failure, not software:

- the pivot block left the left gearbox loaded/seated in a high-stiction
  state (it had just spent four runs as the planted, braked wheel —
  `motorL(0)` with the brake holding it);
- the single 150 ms T_PULSE at pwm 120 (:723–725) did not break it free;
- in T_RUN the corr-boost (≤ +20 over a 85–110 base) topped out at the
  ~120–130 breakaway boundary — sometimes enough (run 4), usually not
  (runs 1–2);
- run 3 (R90) rotated the left motor in the OPPOSITE direction, re-seating
  the gearbox unloaded — after which run 4 broke away instantly and
  reproduced the isolation-battery run byte-for-byte.

This also retro-explains the L45 collapse and the stage-5 "per-start
collapse" statistics: the left wheel sits at the margin of breakaway after
any motion that leaves it loaded, and whether the next turn frees it is a
coin flip the software cannot currently see or force (one pulse, no
per-wheel retry in T_RUN — the retry at :826–831 only fires after 1500 ms
of TOTAL zero progress, which a one-wheel-stalled spin never reaches
because R keeps counting).

Proposed one-line-class fixes (NOT applied, for the offline pass):
1. Per-wheel breakaway retry in the turn controller: if in T_RUN
   `(|cL| == 0 && tsign > 0)` (or the mirrored case) for > 300 ms, re-fire
   `driveTurn(PULSE_PWM)` for that wheel only (a T_PULSE variant), up to
   RETRY_MAX — mirrors what the operator's R90 did mechanically.
2. Or extend the stage-7 ratio detector to `turnStep()` (arm it in
   `turnStart`) so a one-wheel stall inside a spin triggers the existing
   hold/test-pulse machinery.

## Recommendation: (b) PROCEED with the warm-up workaround — acceptance is defensible TODAY, fix lands offline

The acceptance route `G0.8 L90 G1.2 R45 G0.6` starts with a G-command:
`beginDrive` energises BOTH wheels through the straight controller from
standstill, and the stage-7 fault layer IS armed for straights (faultArm in
beginDrive) — a stalled wheel there gets the hold/test-pulse/resync path,
not silent collapse. The exposure is the turns: L90 and R45 are both
left-wheel-critical spins. The evidence says a direction switch before the
turn re-seats the gearbox. So the operator sequence:

**Workaround string (type exactly, robot on the A mark):**

    R5 L5 G0.8 L90 G1.2 R45 G0.6

The throwaway `R5 L5` (5° turns, inside ANG_MIN? — NO: ANG_MIN_DEG = 5.0,
so `R5` is exactly at the boundary and accepted; if the firmware rejects
it, use `R6 L6`) breaks the left wheel free in both directions and ends
with a left-turn breakaway immediately before the route's own L90. If the
queue rejects anything, send `R6` ⏎ `L6` ⏎ then the route string. Log the
workaround in the acceptance template (documented as a workaround, not a
fix). If L90 inside the route still produces asym ≥ 4 counts, ABORT the
acceptance (option a) — that would mean the warm-up is insufficient and
the offline fix must land first.

Option (c)'s fresh-boot test is worth 60 seconds ONLY if the workaround
fails: it answers whether a cold gearbox spins clean (it should, per the
isolation battery), but it does not de-risk the mid-session acceptance
route, which is what matters now.

## What this means for Stage 6 acceptance

The route inherits whatever state the left gearbox is in — not a firmware
variable. It does NOT inherit pivotL mode (the parser proves that), but it
DOES inherit the mechanical stiction left by the pivot block. With the
`R5 L5` (or `R6 L6`) warm-up prepended, the left wheel gets a
direction-cycling breakaway immediately before the route's first turn,
which is precisely the condition under which run 4 reproduced the perfect
spin. Risk remains on R45 (limitation f, collapse-prone even when healthy):
the run-card gate (3/3 in B-zone, no turn off ≥ 4.7°) still applies, and a
collapse there now has a documented mechanical root cause instead of an
unexplained one.
