#!/usr/bin/env python3
# iss lab1 stage 3 - session data tables (Tables 1-4 support)
# usage: python stage3_tables.py [session.csv]
#
# reads session.csv (one row per run):
#   run,gain,dir,cmd,got,L_cnt,R_cnt,odo_L,odo_R,tape_x,tape_y,volt,notes
#
# discipline: every number is REAL (measured), PRED (model), or TODO (not measured).
# rows whose notes contain FAULT:L/R are REAL measurements polluted by an unresolved
# hardware fault (right/left odometry split R/L ~ 1.56-1.82, persists airborne) - they
# are EXCLUDED from all mean/dispersion aggregates and listed explicitly on every run.
# The got-window check (got in [cmd-10, cmd-6] fwd) and the |scale| <= 8.5 mm check
# logic are unchanged.

import sys, os, csv

FAULT_TAG = "FAULT:L/R"

# ---- load ----
path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "lab1_stage3_straight", "session.csv")
rows = []
with open(path, newline="", encoding="utf-8") as fh:
    for r in csv.DictReader(fh):
        rows.append(r)

faulty  = [r for r in rows if FAULT_TAG in r["notes"]]
clean   = [r for r in rows if FAULT_TAG not in r["notes"]]

print("=" * 72)
print("stage3_tables - session", path)
print("=" * 72)
print(f"rows total: {len(rows)}   REAL: {len(rows)}   clean (aggregate-eligible): {len(clean)}   FAULT-excluded: {len(faulty)}")

# ---- excluded-rows list (explicit, every run) ----
print("\n-- excluded rows (FAULT:L/R, kept out of ALL aggregates) --")
for r in faulty:
    print(f"  run {r['run']}: gain={r['gain'] or '-'} dir={r['dir']} cmd={r['cmd'] or '-'} "
          f"got={r['got'] or 'TODO'} L={r['L_cnt']} R={r['R_cnt']} "
          f"R/L={float(r['R_cnt'])/float(r['L_cnt']):.2f}" if r["L_cnt"] else f"  run {r['run']}")
if not faulty:
    print("  (none)")

# ---- sanity derivations that do NOT aggregate (per-row, REAL) ----
K_MM = 5.1087
print("\n-- per-row checks (REAL, no aggregation) --")
for r in rows:
    if r["got"] and r["cmd"]:
        try:
            got, cmd = float(r["got"]), float(r["cmd"])
        except ValueError:
            print(f"  run {r['run']}: got={r['got']} (not numeric - window check skipped)")
            got = None
        if got is not None:
            err = got - cmd
            in_win = -10.0 <= err <= -6.0 if r["dir"] == "fwd" else 6.0 <= err <= 10.0
            print(f"  run {r['run']}: got-window check err={err:+.1f} mm -> {'IN [490,495] window' if in_win else 'OUTSIDE window'}"
                  + ("  [FAULT row - shown, not aggregated]" if FAULT_TAG in r["notes"] else ""))
    if r["odo_L"] and r["R_cnt"] and r["L_cnt"]:
        rl = float(r["R_cnt"]) / float(r["L_cnt"])
        print(f"  run {r['run']}: R/L counts = {rl:.2f}"
              + ("  [FAULT]" if FAULT_TAG in r["notes"] else ""))

# ---- Table 2: lateral deviation (tape) - aggregate over CLEAN rows only ----
print("\n-- Table 2 (lateral deviation, tape) --")
tape_clean = [r for r in clean if r["tape_x"]]
if tape_clean:
    ys = [float(r["tape_y"]) for r in tape_clean if r["tape_y"]]
    print(f"  n={len(ys)} mean|y|={sum(abs(y) for y in ys)/len(ys):.1f} mm")
else:
    print("  TODO (no tape measurements; all tape cells remain TODO until fault resolved)")

# ---- Table 4: controller vs scale error - aggregate over CLEAN rows only ----
print("\n-- Table 4 (controller vs scale error) --")
scale_clean = [r for r in clean if r["got"] and r["tape_x"]]
if scale_clean:
    for r in scale_clean:
        got, cmd, tape = float(r["got"]), float(r["cmd"]), float(r["tape_x"])
        err_ctl = got - cmd
        err_scale = tape - got
        flag = "" if abs(err_scale) <= 8.5 else "  !! k-consistency bound exceeded"
        print(f"  run {r['run']}: err_ctl={err_ctl:+.1f}  err_scale={err_scale:+.1f}{flag}")
else:
    print("  TODO (no clean got+tape pairs; scale error unresolved while fault open)")

# ---- Table 1 note: gain effectiveness blocked by the fault ----
print("\n-- Table 1 (gain tuning) status --")
k_rows = [r for r in rows if r["gain"] in ("0", "1", "3", "6", "0.3", "1.5")]
for r in k_rows:
    tag = "FAULT - excluded, K-effectiveness unresolved" if FAULT_TAG in r["notes"] else "clean"
    print(f"  K{r['gain']} run {r['run']}: L={r['L_cnt']} R={r['R_cnt']} [{tag}]")
print("  K6 observation (REAL, single run, FAULT-tagged): R/L 1.57 with control vs 1.56 without"
      " -> no measurable correction visible under the fault; telemetry re-run pending (T3)")

print("\n-- summary --")
print(f"  aggregates computed over: {len(clean)} clean rows")
print(f"  excluded: {len(faulty)} FAULT rows (listed above)")
print("  discipline: no FAULT row entered any mean/spread; no value estimated or smoothed")
