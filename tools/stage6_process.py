#!/usr/bin/env python
"""stage6_process.py - phase-3 processing for the A -> B acceptance runs.

input : three end-point deviations (x, y) in mm read off the B-cross paper,
        x = along the nominal B direction, y = to the robot's left,
        plus the odometry route length and the tape route length.
        Accepted inputs (pick one):
          --x1/--y1/--x2/--y2/--x3/--y3   raw numbers (original interface)
          --csv FILE                      CSV with header
                                          run,end_pos_x_mm,end_pos_y_mm[,aborts]
          --md FILE                       the stage6_acceptance_template.md
                                          phase-2 markdown table (rows still
                                          reading <<unmeasured>> are skipped)
output: mean shift (systematic component) and spread (random component),
        the guide's separation table, one copyable conclusions block.
        On an empty/unfilled table prints INSUFFICIENT DATA and exits 0
        (never crashes on the template).

usage:
  python tools/stage6_process.py --x1 12 --y1 -8 --x2 9 --y2 -4 --x3 15 --y3 -11 \\
      [--odo 2130] [--tape 2115] [--label "L-shape 1.0/90/1.0"]
  python tools/stage6_process.py --md stage6_acceptance_template.md \\
      [--odo 2600] [--tape 2590]
  python tools/stage6_process.py --csv runs.csv [--odo .. --tape ..]
no input mode -> prints a worked example on session.csv-style numbers so the
operator sees the output shape before entering real values.
"""
import argparse
import csv
import math
import os
import re
import statistics as st

UNMEASURED = "<<unmeasured>>"

# markdown phase-2 row: | run | start_pos | x | y | heading | cmds | aborts |
MD_ROW_RE = re.compile(
    r"^\|\s*(\d+)\s*\|"          # run number
    r"\|(.*?)\|"                  # start_pos
    r"\|(.*?)\|"                  # end_pos_x_mm
    r"\|(.*?)\|"                  # end_pos_y_mm
    r"\|(.*?)\|"                  # heading_err_deg
    r"\|(.*?)\|"                  # commands_executed
    r"\|(.*?)\|", re.IGNORECASE)


def cell(v):
    v = (v or "").strip()
    if not v or v == UNMEASURED or v.upper().startswith("ABORT"):
        return None
    try:
        return float(v)
    except ValueError:
        return None


def points_from_md(path):
    pts, bad = [], []
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = MD_ROW_RE.match(line.strip())
            if not m:
                continue
            run = m.group(1)
            x, y = cell(m.group(3)), cell(m.group(4))
            if x is None or y is None:
                bad.append(run)
            else:
                pts.append((x, y))
    return pts, bad


def points_from_csv(path):
    pts, bad = [], []
    with open(path, encoding="utf-8-sig", newline="") as f:
        rdr = csv.reader(f)
        rows = list(rdr)
    if not rows:
        return pts, bad
    hdr = [h.strip().lower() for h in rows[0]]
    try:
        ix = next(i for i, h in enumerate(hdr) if "x" in h)
        iy = next(i for i, h in enumerate(hdr) if "y" in h)
    except StopIteration:
        # headerless: assume x,y in the first two columns
        ix, iy = 0, 1
        body = rows
    else:
        body = rows[1:]
    for r in body:
        if len(r) <= max(ix, iy):
            continue
        x, y = cell(r[ix]), cell(r[iy])
        if x is None or y is None:
            bad.append(r[0] if r else "?")
        else:
            pts.append((x, y))
    return pts, bad


def report(pts, odo, tape, label):
    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    mx = st.mean(xs); my = st.mean(ys)
    # n=3 -> sample sigma (n-1) is the honest spread; guard n<2
    sx = st.stdev(xs) if len(xs) > 1 else 0.0
    sy = st.stdev(ys) if len(ys) > 1 else 0.0
    shifts = [math.hypot(x - 0.0, y - 0.0) for x, y in pts]  # deviation from B
    mean_abs = st.mean(shifts)
    sigma_abs = st.stdev(shifts) if len(shifts) > 1 else 0.0

    print("=" * 62)
    print(f"stage 6 phase-3 processing: {label}  (n={len(pts)})")
    print(f"runs (x,y) mm from B-cross: {pts}")
    print("-" * 62)
    print(f"along-route mean shift  mx = {mx:+7.1f} mm   sigma_x = {sx:5.1f} mm")
    print(f"lateral   mean shift  my = {my:+7.1f} mm   sigma_y = {sy:5.1f} mm")
    print(f"distance-from-B per run  : {', '.join(f'{d:.0f}' for d in shifts)}")
    print(f"mean |deviation|         = {mean_abs:6.1f} mm")
    print(f"spread sigma(|dev|)      = {sigma_abs:6.1f} mm   (n-1)")
    print("-" * 62)
    print("error separation (guide table):")
    comp = "SYSTEMATIC" if abs(mx) > 2 * sx else "balanced/random-dominated"
    print(f"  along-route: mean {mx:+.1f} vs sigma {sx:.1f} -> {comp}"
          f"  -> fix: k (mmPerCount) re-check" if "SYSTEMATIC" in comp else
          f"  along-route: mean {mx:+.1f} vs sigma {sx:.1f} -> {comp}")
    comp = "SYSTEMATIC" if abs(my) > 2 * sy else "balanced/random-dominated"
    print(f"  lateral:     mean {my:+.1f} vs sigma {sy:.1f} -> {comp}"
          f"  -> fix: wheel track re-check" if "SYSTEMATIC" in comp else
          f"  lateral:     mean {my:+.1f} vs sigma {sy:.1f} -> {comp}")
    print(f"  random (spread) is resolution+slip: tuning will not help - document")
    if odo and tape:
        d = odo - tape
        print("-" * 62)
        print(f"odometry route {odo:.0f} mm vs tape {tape:.0f} mm -> diff {d:+.1f} mm"
              f" ({100.0 * d / tape:+.2f} %)  = CALIBRATION error, not controller error")
    print("-" * 62)
    print("paste-ready conclusions block:")
    print(f"  route: {label}")
    print(f"  end-point deviation from B: mean {mean_abs:.0f} mm, spread {sigma_abs:.0f} mm ({len(pts)} runs)")
    print(f"  systematic: along {mx:+.0f} mm, lateral {my:+.0f} mm;"
          f" random: sigma {sigma_abs:.0f} mm")
    if odo and tape:
        print(f"  odometry-vs-tape scale: {(odo - tape):+.0f} mm on {tape:.0f} mm"
              f" ({100.0 * (odo - tape) / tape:+.2f} %) - calibration, not control")


def main():
    ap = argparse.ArgumentParser(description="stage 6 phase-3 processing")
    ap.add_argument("--x1", type=float); ap.add_argument("--y1", type=float)
    ap.add_argument("--x2", type=float); ap.add_argument("--y2", type=float)
    ap.add_argument("--x3", type=float); ap.add_argument("--y3", type=float)
    ap.add_argument("--csv", default=None, help="CSV: run,end_pos_x_mm,end_pos_y_mm")
    ap.add_argument("--md", default=None, help="stage6_acceptance_template.md phase-2 table")
    ap.add_argument("--odo", type=float, default=None, help="odometry route length mm")
    ap.add_argument("--tape", type=float, default=None, help="tape route length mm")
    ap.add_argument("--label", default="A-B route")
    a = ap.parse_args()

    # 1. markdown template input
    if a.md:
        if not os.path.exists(a.md):
            print(f"ERROR: file not found: {a.md}")
            return 2
        pts, skipped = points_from_md(a.md)
        if not pts:
            print(f"INSUFFICIENT DATA: {a.md} phase-2 table has no filled rows"
                  + (f" (skipped runs: {', '.join(skipped)})" if skipped else ""))
            return 0
        print(f"[{a.md}] parsed {len(pts)} filled run(s)"
              + (f"; skipped unfilled/aborted: {', '.join(skipped)}" if skipped else ""))
        report(pts, a.odo, a.tape, a.label)
        return 0

    # 2. csv input
    if a.csv:
        if not os.path.exists(a.csv):
            print(f"ERROR: file not found: {a.csv}")
            return 2
        pts, skipped = points_from_csv(a.csv)
        if not pts:
            print(f"INSUFFICIENT DATA: {a.csv} has no filled rows"
                  + (f" (skipped: {', '.join(skipped)})" if skipped else ""))
            return 0
        report(pts, a.odo, a.tape, a.label)
        return 0

    # 3. raw numbers
    pts = None
    if None not in (a.x1, a.y1, a.x2, a.y2, a.x3, a.y3):
        pts = [(a.x1, a.y1), (a.x2, a.y2), (a.x3, a.y3)]
    else:
        # worked example, clearly marked - NOT session evidence
        print("[EXAMPLE NUMBERS - replace with the three real runs]")
        pts = [(12.0, -8.0), (9.0, -4.0), (15.0, -11.0)]
        if a.odo is None: a.odo = 2130.0
        if a.tape is None: a.tape = 2115.0
    report(pts, a.odo, a.tape, a.label)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
