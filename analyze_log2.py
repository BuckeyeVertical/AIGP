"""Analysis 2: yaw truth, frame of rate actuation, pitch relation vs heading.

From flight_log2.csv compute:
  - true course angle from dx/dt,dy/dt vs reported yaw and body-vel direction
  - in the lost phase, sign of (reported pitch) vs body-forward accel at
    various headings
"""
import csv
import math

rows = list(csv.DictReader(open("flight_log2.csv")))
f = lambda r, k: float(r[k])

print("t     yaw_rep  course(dxdy)  body_vel_ang  yaw_implied=course-bodyang  vbx    vby   pitch  dvbx/dt")
N = 8
for i in range(N, len(rows) - N, 25):
    r0, r1, r2 = rows[i - N], rows[i], rows[i + N]
    dt = f(r2, "t") - f(r0, "t")
    if dt <= 0:
        continue
    dx = (f(r2, "x") - f(r0, "x")) / dt
    dy = (f(r2, "y") - f(r0, "y")) / dt
    speed = math.hypot(dx, dy)
    vbx, vby = f(r1, "vbx"), f(r1, "vby")
    bspeed = math.hypot(vbx, vby)
    if speed < 0.5 or bspeed < 0.5:
        continue
    course = math.atan2(dy, dx)
    bang = math.atan2(vby, vbx)
    implied = (course - bang + math.pi) % (2 * math.pi) - math.pi
    dvbx = (f(r2, "vbx") - f(r0, "vbx")) / dt
    print(
        "%5.1f  %+6.2f   %+6.2f        %+6.2f       %+6.2f               %+5.2f  %+5.2f  %+5.2f  %+5.2f"
        % (f(r1, "t"), f(r1, "yaw"), course, bang, implied, vbx, vby,
           f(r1, "pitch"), dvbx)
    )
