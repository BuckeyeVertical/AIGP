"""One-off analysis of flight_log.csv: extract true sign conventions."""
import csv

rows = list(csv.DictReader(open("flight_log.csv")))
f = lambda r, k: float(r[k])

print("t      pitch_rep  dvbx/dt  pitch_q   dpitch/dt   roll_rep  dvby/dt  roll_q")
for i in range(10, len(rows) - 16, 30):
    r = rows[i]
    r2 = rows[i + 15]
    dt = f(r2, "t") - f(r, "t")
    if dt <= 0:
        continue
    dvbx = (f(r2, "vbx") - f(r, "vbx")) / dt
    dvby = (f(r2, "vby") - f(r, "vby")) / dt
    dp = (f(r2, "pitch") - f(r, "pitch")) / dt
    print(
        "%5.1f  %+8.3f  %+7.2f  %+7.3f  %+9.3f   %+7.3f  %+6.2f  %+6.3f"
        % (
            f(r, "t"), f(r, "pitch"), dvbx, f(r, "pitch_q"), dp,
            f(r, "roll"), dvby, f(r, "roll_q"),
        )
    )

# correlation summary over whole log
import statistics
ps, axs, rs, ays, pqs, dps, rqs, drs = [], [], [], [], [], [], [], []
for i in range(1, len(rows) - 1):
    r0, r1, r2 = rows[i - 1], rows[i], rows[i + 1]
    dt = f(r2, "t") - f(r0, "t")
    if dt <= 0:
        continue
    ps.append(f(r1, "pitch"))
    axs.append((f(r2, "vbx") - f(r0, "vbx")) / dt)
    rs.append(f(r1, "roll"))
    ays.append((f(r2, "vby") - f(r0, "vby")) / dt)
    pqs.append(f(r1, "pitch_q"))
    dps.append((f(r2, "pitch") - f(r0, "pitch")) / dt)
    rqs.append(f(r1, "roll_q"))
    drs.append((f(r2, "roll") - f(r0, "roll")) / dt)


def corr(a, b):
    ma, mb = statistics.mean(a), statistics.mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = sum((x - ma) ** 2 for x in a) ** 0.5
    db = sum((y - mb) ** 2 for y in b) ** 0.5
    return num / (da * db) if da and db else 0.0


print()
print("corr(reported pitch, body-fwd accel)  =", round(corr(ps, axs), 3))
print("corr(reported roll,  body-right accel)=", round(corr(rs, ays), 3))
print("corr(pitch_q sent, d(reported pitch)) =", round(corr(pqs, dps), 3))
print("corr(roll_q sent,  d(reported roll))  =", round(corr(rqs, drs), 3))
