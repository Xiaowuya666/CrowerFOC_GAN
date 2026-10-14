import csv
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parent / "axis_feature_results"
src = root / "20261009_speed_limit_diagnostic.csv"
out = root / "20261009_speed_limit_diagnostic.png"
with src.open(newline="", encoding="utf-8-sig") as f:
    rows = list(csv.DictReader(f))
for r in rows:
    for k in ("t_s", "target_rpm", "vel_rpm", "vq_V", "voltage_limit_V", "iq_A", "state", "fault"):
        r[k] = float(r[k])
runs = [r for r in rows if r["state"] == 3 and r["fault"] == 0]
t = np.array([r["t_s"] for r in runs]); target = np.array([r["target_rpm"] for r in runs])
speed = np.array([r["vel_rpm"] for r in runs]); vq = np.array([r["vq_V"] for r in runs])
vlim = np.array([r["voltage_limit_V"] for r in runs]); iq = np.array([r["iq_A"] for r in runs])
plt.rcParams.update({"font.family": "Microsoft YaHei", "axes.grid": True, "grid.alpha": .25})
fig, ax = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
ax[0].plot(t, target, "k--", lw=1.2, label="速度指令")
ax[0].plot(t, speed, color="tab:blue", lw=1, label="实测关节输出速度")
ax[0].set_ylabel("速度 (rpm)"); ax[0].legend(loc="upper left")
ax[0].set_title("低电流速度分档：100 rpm附近开始受限，150–230 rpm请求未能跟随")
ax[1].plot(t, vq, color="tab:orange", label="Vq")
ax[1].plot(t, vlim, "r--", label="电压限幅")
ax[1].set_ylabel("电压 (V)"); ax[1].legend(loc="upper left")
ax[2].plot(t, iq, color="tab:green", label="Iq")
ax[2].axhline(1.0, color="k", ls=":", label="临时运行限流 1 A")
ax[2].set_ylabel("Iq (A)"); ax[2].set_xlabel("记录时间 (s)"); ax[2].legend(loc="upper left")
for a in ax: a.set_xlim(float(t.min()), float(t.max()))
fig.suptitle("CrowerFOC：24.46 V母线、9:1减速、无载速度上限诊断（2026-10-09）", fontsize=14)
fig.savefig(out, dpi=170)
print(out)
