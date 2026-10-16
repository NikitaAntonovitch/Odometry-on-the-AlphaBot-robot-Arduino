#!/usr/bin/env python3
# iss lab1 stage 2 - data processing for the measurements taken at the lab
# usage: python stage2_process.py data.txt   (format below; a dummy data.txt is generated if missing)
# data.txt format - one section, lines of numbers, '#' starts a comment:
#
#   [k]                      <- calibration of k
#   L_sec = 1000             <- marked section length mm
#   slots = 20               <- counted slots S (both discs equal; if not, put 2 values and say so)
#   wheel_d = 65.0           <- calliper diameter mm
#   1  392 388               <- run 1 fwd: counts L, counts R
#   2  395 391
#   3  390 392
#   4 -391 -394              <- reverse run (sign negative)
#
#   [pwm]                    <- static characteristic
#   50  120 128
#   90  210 226              <- pwm, speed L mm/s, speed R mm/s
#   130 292 312
#   170 356 382
#   210 420 448
#
# outputs: k table with mean/spread, comparison vs geometry, asymmetry %,
#          and two plots: pwm->speed, asymmetry.

import sys, os
import statistics as st

SAMPLE = """[k]
L_sec = 1000
slots = 20
wheel_d = 65.0
1  392 388
2  395 391
3  390 392
4 -391 -394

[pwm]
50  120 128
90  210 226
130 292 312
170 356 382
210 420 448
"""

def load(path):
    if not os.path.exists(path):
        open(path, "w").write(SAMPLE)
        print(f"no data file found - wrote dummy sample to {path}, edit it and rerun")
        sys.exit(0)
    k = {"runs": []}
    pwm = []
    sec = None
    for raw in open(path):
        line = raw.split("#")[0].strip()
        if not line:
            continue
        if line.startswith("[k]"):
            sec = k; continue
        if line.startswith("[pwm]"):
            sec = None; sec_pwm = True; continue
        if "=" in line:
            key, val = line.split("=", 1)
            key, val = key.strip(), float(val.strip())
            if sec is not None:
                sec[key] = val
            continue
        parts = line.split()
        nums = [float(p) for p in parts]
        if sec is not None:
            sec["runs"].append(nums)
        else:
            pwm.append(nums)
    return k, pwm

def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "data.txt"
    k_data, pwm_rows = load(path)

    S = int(k_data["slots"])
    N_rev = 2 * S
    k_geom = 3.14159265 * k_data["wheel_d"] / N_rev

    print("== calibration of k ==")
    print(f"slots S={S}  N_rev(CHANGE)={N_rev}  wheel D={k_data['wheel_d']} mm  k_geom={k_geom:.3f} mm/count")
    print(f"marked section: {k_data['L_sec']:.0f} mm")
    kL, kR = [], []
    for run in k_data["runs"]:
        n, cntl, cntr = int(run[0]), run[1], run[2]
        aL, aR = abs(cntl), abs(cntr)
        if aL: kL.append(k_data["L_sec"] / aL)          # k = section length / counts over it
        if aR: kR.append(k_data["L_sec"] / aR)
        print(f"run {n}: cnt L={cntl:.0f} R={cntr:.0f} -> k_L={kL[-1] if kL else float('nan'):.3f} k_R={kR[-1] if kR else float('nan'):.3f} mm/count")
    allk = kL + kR
    k_mean = st.mean(allk)
    k_spread = (max(allk) - min(allk)) / k_mean * 100
    print(f"\nk_mean = {k_mean:.3f} mm/count   k_geom = {k_geom:.3f}   diff = {(k_mean-k_geom)/k_geom*100:+.2f} %")
    print(f"spread = {k_spread:.2f} %  ({'OK, <=2%' if k_spread <= 2 else 'TOO BIG - method flaw, do not blame hardware'})")
    print(f"-> adopted K_MM = {k_mean:.4f}   (quantisation alone is +/-{k_mean/2:.2f} mm)")

    print("\n== static pwm -> speed characteristic ==")
    print("pwm    L mm/s    R mm/s    asym %")
    pwms, spL, spR, asym = [], [], [], []
    for row in pwm_rows:
        p, l, r = row
        a = (r - l) / ((l + r) / 2) * 100
        pwms.append(p); spL.append(l); spR.append(r); asym.append(a)
        print(f"{p:5.0f}  {l:7.1f}  {r:7.1f}  {a:+6.1f}")
    big = [a for a in asym if abs(a) > 15]
    sgn = [1 if a > 0 else -1 for a in asym if a != 0]
    if big:
        print("WARNING: asymmetry >15% somewhere - check mechanics, controller will not fix it")
    if len(set(sgn)) > 1:
        print("WARNING: asymmetry changes sign across the pwm range - check mechanics")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(10, 4))
        ax[0].plot(pwms, spL, "o-", label="left")
        ax[0].plot(pwms, spR, "s-", label="right")
        ax[0].set_xlabel("PWM"); ax[0].set_ylabel("speed mm/s")
        ax[0].set_title("stage2: pwm -> speed"); ax[0].grid(True); ax[0].legend()
        ax[1].plot(pwms, asym, "d-")
        ax[1].axhline(0, color="k", lw=0.5); ax[1].axhline(15, color="r", ls=":"); ax[1].axhline(-15, color="r", ls=":")
        ax[1].set_xlabel("PWM"); ax[1].set_ylabel("asymmetry %")
        ax[1].set_title("stage2: wheel asymmetry"); ax[1].grid(True)
        fig.tight_layout()
        out = os.path.join(os.path.dirname(os.path.abspath(path)), "stage2_characteristic.png")
        fig.savefig(out, dpi=120)
        print(f"\nplots written: {out}")
    except ImportError:
        print("\nmatplotlib not installed - skipped plots (pip install matplotlib)")

if __name__ == "__main__":
    main()
