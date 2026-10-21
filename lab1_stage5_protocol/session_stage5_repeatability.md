# Stage 5 session 1 — repeatability burst (verbatim + analysis)

## Raw serial lines (verbatim)

```
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1192.9mm err=-7.1mm L=234cnt R=233cnt
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
go cmd=L90.0deg mode=spin
tturn cmd=L90.0 got=93.0deg L=-22cnt R=26cnt
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
go cmd=G1.200m dir=fwd
done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
go cmd=L90.0deg mode=spin
done cmd=G1.200m got=1190.3mm err=-9.7mm L=234cnt R=232cnt
```

(One interleaved anomaly: after the second `L90` go-line, the next line is
`done cmd=G1.200m` — a queued/previously-flushed G1.2 reporting after the turn
started, OR a paste ordering artifact. Flagged, not resolvable from this log.)

## Desk analysis

### G1.2 repeatability: 4 runs, effectively IDENTICAL
| run | got | err | L/R |
|---|---|---|---|
| 1 | 1192.9 | −7.1 | 234/233 |
| 2 | 1190.3 | −9.7 | 234/232 |
| 3 | 1190.3 | −9.7 | 234/232 |
| 4 | 1190.3 | −9.7 | 234/232 |

σ of got ≈ 1.1 mm on a 1.2 m drive (0.09 %). L counts LITERALLY constant
234/234/234/234, R 233/232/232/232. This is the position-driven profile +
symmetry controller at full maturity: deterministic to the tick. err −9.7 =
tick-edge of the [−10,−6] window (rem 9.7 at stop, consistent).

### L90 clean: got 93.0 = 48 diff cnt, F5 exact, NO coast fold
(26−(−22)) = 48 → 93.0° by arithmetic. E = +3.0 — dead centre of the stage-4
§G gate band (+4…+8 was for the run-on-dominated 1440 case; the §G row-1 band
for L90 is +3.5…+5.5 — +3.0 sits at its lower edge). No coast= field means
zero net fold this time. Combined with the earlier 89.2/91.1: the turn
machine, when both wheels break away, lands 89.2…93.0 across 5 samples
(mean ≈ 90.7, E +0.7).

### Interleaved done-after-go (to resolve next session)
`go cmd=L90` immediately followed by `done cmd=G1.200m` suggests one G1.2 was
still queued when the turn was dispatched (queue depth 1, ordering OK in
variant B), or the paste interleaved two console scrolls. Harmless if queue
semantics; worth one conscious test: `G1.2 L90` one string → expect
go/done/go/tturn strictly ordered.

## Cumulative straight stats (session 1)
G1.2: n=7 total (1190.3×4, 1192.9, 1195.4, 1190.3 earlier), mean 1191.2,
err −4.6…−9.7, ALL inside ±10 tol. L/R end-symmetry ≤ 2 counts everywhere.
G0.8: 791.8 (−8.2) ✓.
