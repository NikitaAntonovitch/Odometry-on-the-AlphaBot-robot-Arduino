#!/usr/bin/env python
"""stage6_process.py - phase-3 processing for the A -> B acceptance runs.

input : three end-point deviations (x, y) in mm read off the B-cross paper,
        x = along the nominal B direction, y = to the robot's left,
        plus the odometry route length and the tape route length.
output: mean shift (systematic component) and spread (random component),
        the guide's separation table, one copyable conclusions block.

usage:
  python tools/stage6_process.py --x1 12 --y1 -8 --x2 9 --y2 -4 --x3 15 --y3 -11 \
      [--odo 2130] [--tape 2115] [--label "L-shape 1.0/90/1.0"]
no args -> prints a worked example on the session.csv-style numbers so the
operator sees the output shape before entering real values.
"""
import argparse
import math
import statistics as st

def main():
    ap = argparse.ArgumentParser(description="stage 6 phase-3 processing")
    ap.add_argument("--x1", type=float); ap.add_argument("--y1", type=float)
    ap.add_argument("--x2", type=float); ap.add_argument("--y2", type=float)
    ap.add_argument("--x3", type=float); ap.add_argument("--y3", type=float)
    ap.add_argument("--odo", type=float, default=None, help="odometry route length mm")
    ap.add_argument("--tape", type=float, default=None, help="tape route length mm")
    ap.add_argument("--label", default="A-B route")
    a = ap.parse_args()

    pts = None
    if None not in (a.x1, a.y1, a.x2, a.y2, a.x3, a.y3):
        pts = [(a.x1, a.y1), (a.x2, a.y2), (a.x3, a.y3)]
    else:
        # worked example, clearly marked - NOT session evidence
        print("[EXAMPLE NUMBERS - replace with the three real runs]")
        pts = [(12.0, -8.0), (9.0, -4.0), (15.0, -11.0)]
        if a.odo is None: a.odo = 2130.0
        if a.tape is None: a.tape = 2115.0

    xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    mx = st.mean(xs); my = st.mean(ys)
    # n=3 -> sample sigma (n-1) is the honest spread; guard n<2
    sx = st.stdev(xs) if len(xs) > 1 else 0.0
    sy = st.stdev(ys) if len(ys) > 1 else 0.0
    shifts = [math.hypot(x - 0.0, y - 0.0) for x, y in pts]  # deviation from B
    mean_abs = st.mean(shifts)
    sigma_abs = st.stdev(shifts) if len(shifts) > 1 else 0.0

    print("=" * 62)
    print(f"stage 6 phase-3 processing: {a.label}")
    print(f"runs (x,y) mm from B-cross: {pts}")
    print("-" * 62)
    print(f"along-route mean shift  mx = {mx:+7.1f} mm   sigma_x = {sx:5.1f} mm")
    print(f"lateral   mean shift  my = {my:+7.1f} mm   sigma_y = {sy:5.1f} mm")
    print(f"distance-from-B per run  : {', '.join(f'{d:.0f}' for d in shifts)}")
    print(f"mean |deviation|         = {mean_abs:6.1f} mm")
    print(f"spread sigma(|dev|)      = {sigma_abs:6.1f} mm   (n=3, n-1)")
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
    if a.odo and a.tape:
        d = a.odo - a.tape
        print("-" * 62)
        print(f"odometry route {a.odo:.0f} mm vs tape {a.tape:.0f} mm -> diff {d:+.1f} mm"
              f" ({100.0 * d / a.tape:+.2f} %)  = CALIBRATION error, not controller error")
    print("-" * 62)
    print("paste-ready conclusions block:")
    print(f"  route: {a.label}")
    print(f"  end-point deviation from B: mean {mean_abs:.0f} mm, spread {sigma_abs:.0f} mm (3 runs)")
    print(f"  systematic: along {mx:+.0f} mm, lateral {my:+.0f} mm;"
          f" random: sigma {sigma_abs:.0f} mm")
    if a.odo and a.tape:
        print(f"  odometry-vs-tape scale: {(a.odo - a.tape):+.0f} mm on {a.tape:.0f} mm"
              f" ({100.0 * (a.odo - a.tape) / a.tape:+.2f} %) - calibration, not control")

if __name__ == "__main__":
    main()
