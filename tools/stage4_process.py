#!/usr/bin/env python
"""stage4_process.py - turn-accuracy processing for the stage-4 tables.

input : the filled stage4_accuracy_template.md (markdown tables). Only
        stdlib is used (re, statistics). The script runs on the EMPTY
        template too: every cell still reading <<unmeasured>> (or ABORT)
        is skipped, and a group with no usable rows prints INSUFFICIENT
        DATA instead of crashing.

output: per (mode, angle, dir) group: n, mean err_odom (deg), spread
        sigma (deg, n-1), mean |err_tape| if protractor data exists,
        plus the checkpoint-4 verdict rules:
          - 45 deg gate: worst |mean err_tape| <= 2.25 deg (5 % of 45)
          - mean error growing with angle  -> track under-calibrated (scale)
          - mean error constant, sigma big -> run-on / stop transient
          - sigma never shrinks            -> sensor resolution limit

usage:
  python tools/stage4_process.py stage4_accuracy_template.md
  python tools/stage4_process.py --file stage4_accuracy_template.md
"""
import argparse
import os
import re
import statistics as st
import sys

UNMEASURED = "<<unmeasured>>"

ROW_RE = re.compile(
    r"^\|\s*(\d+(?:\.\d+)?)\s*"      # angle
    r"\|\s*([LR])\s*"                # dir
    r"\|\s*(\d+)\s*"                 # rep
    r"\|\s*(.*?)\s*"                 # target_deg
    r"\|\s*(.*?)\s*"                 # achieved_odometry_deg
    r"\|\s*(.*?)\s*"                 # measured_protractor_deg
    r"\|\s*(.*?)\s*"                 # N_L
    r"\|\s*(.*?)\s*"                 # N_R
    r"\|\s*(.*?)\s*"                 # coast_cnt
    r"\|\s*(.*?)\s*"                 # err_odom
    r"\|\s*(.*?)\s*\|"               # err_tape
)
HDR_RE = re.compile(r"^\|\s*angle\s*\|\s*dir\s*\|", re.IGNORECASE)
SEP_RE = re.compile(r"^\|[\s:|-]+\|$")
MODE_RE = re.compile(r"^##\s+Table\s+(\d+)\s*[—-]\s*(SPIN|PIVOT)\b", re.IGNORECASE)


def cell(v):
    """numeric or None; ABORT/unmeasured/blank -> None."""
    v = (v or "").strip()
    if not v or v == UNMEASURED or v.upper().startswith("ABORT"):
        return None
    v = v.replace("\u2212", "-")   # U+2212: reports/editors write −1.50, float() only knows '-'
    try:
        return float(v)
    except ValueError:
        return None


def signed_target(angle, dirc):
    return angle if dirc == "L" else -angle


def parse(path):
    rows = []          # dicts
    mode = None
    with open(path, encoding="utf-8") as f:
        for line in f:
            m = MODE_RE.match(line.strip())
            if m:
                mode = m.group(2).lower()
                continue
            if HDR_RE.match(line) or SEP_RE.match(line):
                continue
            m = ROW_RE.match(line.strip())
            if not m or mode is None:
                continue
            angle, dirc, rep, tgt, ach, prot = m.group(1), m.group(2).upper(), m.group(3), m.group(4), m.group(5), m.group(6)
            nl, nr, coast, eo, et = m.group(7), m.group(8), m.group(9), m.group(10), m.group(11)
            a = float(angle)
            ach_v, prot_v = cell(ach), cell(prot)
            # err_odom: use the filled cell; if blank but achieved exists, compute
            eo_v = cell(eo)
            if eo_v is None and ach_v is not None:
                eo_v = ach_v - signed_target(a, dirc)
            et_v = cell(et)
            if et_v is None and prot_v is not None:
                et_v = prot_v - signed_target(a, dirc)
            rows.append({
                "mode": mode, "angle": a, "dir": dirc, "rep": int(rep),
                "aborted": ach.strip().upper().startswith("ABORT"),
                "err_odom": eo_v, "err_tape": et_v,
                "nl": cell(nl), "nr": cell(nr), "coast": cell(coast),
            })
    return rows


def main():
    ap = argparse.ArgumentParser(description="stage 4 turn-accuracy processing")
    ap.add_argument("template", nargs="?", default="stage4_accuracy_template.md")
    ap.add_argument("--file", dest="template_opt", default=None)
    args = ap.parse_args()
    path = args.template_opt or args.template
    if not os.path.exists(path):
        print(f"ERROR: file not found: {path}")
        return 2

    rows = parse(path)
    groups = {}
    for r in rows:
        if r["aborted"]:
            continue
        if r["err_odom"] is None and r["err_tape"] is None:
            continue
        groups.setdefault((r["mode"], r["angle"], r["dir"]), []).append(r)

    print("=" * 72)
    print(f"stage 4 turn accuracy: {path}")
    print("=" * 72)
    if not rows:
        print("no table rows parsed - is this the right template?")
        return 2
    if not groups:
        print("no filled rows yet - every value is still <<unmeasured>>.")
        print()
        print(f"{'mode':6} {'angle':>6} {'dir':4} {'n':>3}   status")
        for (mode, angle, dirc) in sorted(groups.keys() | {(r['mode'], r['angle'], r['dir']) for r in rows}):
            print(f"{mode:6} {angle:6.0f} {dirc:4} {0:>3}   INSUFFICIENT DATA")
        print()
        print("fill the tables on the floor and re-run. aborts stay in the")
        print("table as ABORT rows; they are counted, not deleted.")
        return 0

    # group-level table
    print(f"{'mode':6} {'angle':>6} {'dir':4} {'n':>3} {'n_abort':>8} "
          f"{'mean err_odom':>13} {'sigma':>6} {'mean err_tape':>13}   verdict")
    gate45 = True
    gate45_seen = False
    means = {}
    for key in sorted(groups):
        mode, angle, dirc = key
        g = groups[key]
        all_rows_for_cell = [r for r in rows
                             if (r["mode"], r["angle"], r["dir"]) == key]
        n_ab = sum(1 for r in all_rows_for_cell if r["aborted"])
        eo = [r["err_odom"] for r in g if r["err_odom"] is not None]
        et = [r["err_tape"] for r in g if r["err_tape"] is not None]
        n = len(g)
        if n < 2:
            print(f"{mode:6} {angle:6.0f} {dirc:4} {n:>3} {n_ab:>8} "
                  f"{'-':>13} {'-':>6} {'-':>13}   INSUFFICIENT DATA (need >= 2)")
            continue
        m_eo = st.mean(eo) if eo else None
        s_eo = st.stdev(eo) if len(eo) > 1 else 0.0
        m_et = st.mean(et) if et else None
        means.setdefault(mode, []).append((angle, m_eo, m_et, s_eo))
        verdict = ""
        if angle == 45.0 and m_et is not None:
            gate45_seen = True
            if abs(m_et) > 2.25:
                verdict = "FAILS 45deg 5% gate"
                gate45 = False
            else:
                verdict = "45deg gate OK"
        print(f"{mode:6} {angle:6.0f} {dirc:4} {n:>3} {n_ab:>8} "
              f"{(f'{m_eo:+.2f}' if m_eo is not None else '-'):>13} {s_eo:6.2f} "
              f"{(f'{m_et:+.2f}' if m_et is not None else '-'):>13}   {verdict}")

    # trend verdicts per mode
    print("-" * 72)
    for mode, lst in means.items():
        lst = sorted(lst)
        if len(lst) < 3:
            print(f"{mode}: trend verdict needs >= 3 angles filled - INSUFFICIENT DATA")
            continue
        abs_means = [(a, (abs(m_et) if m_et is not None else None), s)
                     for (a, _meo, m_et, s) in lst]
        filled = [(a, m, s) for (a, m, s) in abs_means if m is not None]
        if len(filled) < 3:
            print(f"{mode}: trend verdict needs protractor (tape) means - INSUFFICIENT DATA")
            continue
        growth = filled[-1][1] - filled[0][1]
        sigma_mean = st.mean([s for (_a, _m, s) in filled])
        if growth > 1.0 and filled[-1][1] > 2.25:
            print(f"{mode}: |err| grows with angle ({growth:+.1f} deg across the table) "
                  f"-> SCALE: track under-calibrated, recalibrate track from these runs")
        elif max(s for (_a, _m, s) in filled) > 2 * (st.mean([m for (_a, m, _s) in filled]) or 1):
            print(f"{mode}: mean small, sigma large -> RUN-ON / stop transient (control, not scale)")
        else:
            print(f"{mode}: no dominant scale trend; mean spread sigma ~{sigma_mean:.1f} deg - "
                  f"if this stays after tuning: sensor resolution limit (document, do not chase)")

    if gate45_seen:
        print("-" * 72)
        print(f"checkpoint-4 45deg gate (worst-case |mean err_tape| <= 2.25 deg): "
              f"{'PASS' if gate45 else 'FAIL'} - the assignment's 'at least 45 deg' "
              f"requirement is {'met' if gate45 else 'NOT met on this data'}")
    print()
    print("reminder: ABORT rows are evidence (cell with >= 3 aborts of 5 = stop,")
    print("charge/inspect, do not silently re-run).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
