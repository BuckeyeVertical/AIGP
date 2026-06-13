"""Fit the tilt->acceleration mapping from flight logs (numpy version).

Candidates: a_world = g * Rz(s*yaw) @ [c_f*pitch_rep, c_r*roll_rep]
(vector = (fwd-ish, right-ish) tilt). Score by residual vs acceleration
obtained from deduped, interpolated, smoothed x/y.
"""
import csv
import math
import sys

import numpy as np

fname = sys.argv[1] if len(sys.argv) > 1 else "flight_log5.csv"
rows = list(csv.DictReader(open(fname)))
t = np.array([float(r["t"]) for r in rows])
x = np.array([float(r["x"]) for r in rows])
y = np.array([float(r["y"]) for r in rows])
yaw = np.array([float(r["yaw"]) for r in rows])
pitch = np.array([float(r["pitch"]) for r in rows])
roll = np.array([float(r["roll"]) for r in rows])

# Dedupe on changing position (telemetry updates slower than the loop).
keep = np.concatenate(([True], (np.diff(x) != 0) | (np.diff(y) != 0)))
tk, xk, yk = t[keep], x[keep], y[keep]

# Uniform resample at 20 Hz, then smooth.
tu = np.arange(tk[0], tk[-1], 0.05)
xu = np.interp(tu, tk, xk)
yu = np.interp(tu, tk, yk)
win = 11
ker = np.ones(win) / win
xs = np.convolve(xu, ker, mode="valid")
ys = np.convolve(yu, ker, mode="valid")
ts = tu[win // 2 : win // 2 + len(xs)]

vn = np.gradient(xs, ts)
ve = np.gradient(ys, ts)
vn = np.convolve(vn, ker, mode="same")
ve = np.convolve(ve, ker, mode="same")
an = np.gradient(vn, ts)
ae = np.gradient(ve, ts)

# Attitude at those times.
yaw_s = np.interp(ts, t, np.unwrap(yaw))
p_s = np.interp(ts, t, pitch)
r_s = np.interp(ts, t, roll)

# Trim ends and low-information samples.
m = (ts > ts[0] + 1) & (ts < ts[-1] - 1)
an, ae, yaw_s, p_s, r_s = an[m], ae[m], yaw_s[m], p_s[m], r_s[m]

g = 9.81
print(f"{len(an)} samples from {fname}; accel rms = {np.sqrt(np.mean(an**2+ae**2)):.2f} m/s^2")
print("s(yaw)  c_fwd  c_right   mean|resid|   corr")
best = None
for s in (1, -1):
    for cf in (1, -1):
        for cr in (1, -1):
            fwd = cf * p_s
            right = cr * r_s
            cy, sy = np.cos(s * yaw_s), np.sin(s * yaw_s)
            pan = g * (cy * fwd - sy * right)
            pae = g * (sy * fwd + cy * right)
            res = float(np.mean(np.hypot(an - pan, ae - pae)))
            corr = float(
                np.sum(an * pan + ae * pae)
                / (np.sqrt(np.sum(pan**2 + pae**2)) * np.sqrt(np.sum(an**2 + ae**2)))
            )
            print("  %+d      %+d      %+d      %8.3f     %+6.3f" % (s, cf, cr, res, corr))
            if best is None or corr > best[0]:
                best = (corr, s, cf, cr)
print("BEST by corr: s=%+d c_fwd=%+d c_right=%+d  corr=%.3f" % (best[1], best[2], best[3], best[0]))
