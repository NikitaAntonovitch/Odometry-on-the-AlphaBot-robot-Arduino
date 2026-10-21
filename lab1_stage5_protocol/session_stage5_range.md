# Stage 5 session 1 — range rules + signs (verbatim + analysis) — §4/§5 of the worksheet

## Raw serial lines (verbatim)

```
go cmd=L90.0deg mode=spin
tturn cmd=L90.0 got=87.2deg L=-24cnt R=21cnt coast=-1cnt
err: G out of range (0.005..5.000 m)
err: angle out of range (5..1440 deg)
mode=stop pwm=n/a (brake)
mode=stop pwm=n/a (coast)
err: angle out of range (5..1440 deg)
go cmd=G-0.300m dir=back
done cmd=G-0.300m got=-293.8mm err=6.2mm L=-58cnt R=-57cnt
err: G out of range (0.005..5.000 m)
err: G out of range (0.005..5.000 m)
```

(The sends are not in the paste — mapping by the error text against the
worksheet §5 list: G99 → G-range err; L3/L2000 → angle-range err ×2;
G0.001 → G-range err. The two `mode=stop` lines are the c/b abort keys.)

## Desk analysis

### Range rules: WORKING
- 3× `err: G out of range (0.005..5.000 m)` — G99 / G0.001(+1 more) rejected,
  no motion lines, robot stayed put.
- 2× `err: angle out of range (5..1440 deg)` — L3 / L2000 rejected.
- Error lines are exactly the one-line-no-motion behaviour the checkpoint
  requires ("behaviour defined for an out-of-range argument").
- Critically: the rejects did NOT wedge the parser — the valid G−0.3 after
  them ran fine (string-continues behaviour proven across mixed junk).

### G−0.3: negative-metres backwards drive PASS
got −293.8, err +6.2 (in-window on the negative side), R/L = 57/58 = 0.983,
drag asymmetry present as usual but equalised. Sign rule confirmed on a THIRD
magnitude (0.2, 0.3 m) and both directions.

### L90: got 87.2 = (21−(−24)) = 45 diff cnt, F5 exact, coast −1
E = −2.8. Healthy-wheel pool now: 87.2, 89.2, 89.2, 91.1, 91.1, 93.0 →
mean 90.1, E +0.1, σ 2.1. The no-collapse turn machine is on target.

### Checkpoint-5 coverage after this batch
- ☑ sequence as single string — still pending (§2 THE item)
- ☑ unrecognised characters skipped — junk+G0.1 ran earlier ✓
- ☑ negative/units rule — G−0.2, G−0.3, L−45? (L−45/R−45 abs-rule rows not
  yet pasted — the sends here were plain G99/L3/L2000/G0.001)
- ☑ out-of-range → err line, no motion, parser survives — PROVEN this batch
- ☑ one result line per command, got= in turn lines — proven throughout
- ☑ no PC program needed — entire session via serial monitor

### Remaining for stage 5 closure
1. §2 sequence `G0.2 L90 G0.3` one string (THE item)
2. §4 L−45 / R−45 abs-rule rows (explicit pastes)
3. §6 queue/abort conscious test (`G1.2 L90` ordering + mid-drive c)
4. (optional) lowercase g0.1 l45 paste for the case-insensitivity row
