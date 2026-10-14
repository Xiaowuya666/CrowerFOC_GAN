"""Plot saved axis feature regressions in the public rpm/degree units."""
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "axis_feature_results"
FEATURE = OUT / "20261009_174134_features.json"
CONTROL = OUT / "20261009_174759_control.json"


def segments(rows, predicate):
    result, active, last_t = [], [], None
    for row in rows:
        if predicate(row) and (last_t is None or row["t"] - last_t < 0.12):
            active.append(row)
        else:
            if len(active) > 30:
                result.append(active)
            active = [row] if predicate(row) else []
        last_t = row["t"]
    if len(active) > 30:
        result.append(active)
    return result


def load(path):
    d = json.loads(path.read_text(encoding="utf-8"))
    t0 = next((x["t"] for x in d["samples"] if "t" in x), 0)
    for x in d["samples"]:
        x["time_s"] = x.get("t", t0) - t0
    return d


feature, control = load(FEATURE), load(CONTROL)
plt.rcParams.update({"font.family": "Microsoft YaHei", "axes.grid": True, "grid.alpha": .25})
fig, ax = plt.subplots(3, 1, figsize=(12, 10), constrained_layout=True)
velocity_runs = segments(feature["samples"], lambda x: x["state"] == 3 and x["mode"] == 1)
for i, run in enumerate(velocity_runs[:2]):
    t = np.array([x["time_s"] for x in run]); rpm = np.array([x["vel_rpm"] for x in run])
    iq = np.array([x["iq_A"] for x in run]); target = 30 if np.mean(rpm) > 0 else -30
    ax[0].plot(t, rpm, label=f"实测（指令 {target:+d} rpm）")
    ax[0].axhline(target, ls="--", alpha=.45)
    ax[1].plot(t, iq, label=f"Iq（指令 {target:+d} rpm）")
ax[0].set_ylabel("速度 (rpm)"); ax[0].set_title("USB低电流速度闭环：±30 rpm")
ax[1].set_ylabel("Iq (A)"); ax[1].set_title("速度测试电流")
ax[0].legend(); ax[1].legend()

position_runs = segments(feature["samples"], lambda x: x["state"] == 3 and x["mode"] == 2)
if position_runs:
    run = position_runs[-1]
    t = np.array([x["time_s"] for x in run]); pos = np.array([x["pos_deg"] for x in run])
    target = pos[0] + 20
    ax[2].plot(t, pos, label="机械位置")
    ax[2].axhline(target, color="tab:red", ls="--", label="约 +20°目标")
    ax[2].set_ylabel("位置 (°)"); ax[2].set_title(
        f"梯形位置命令：记录误差 {feature['checks']['position_error_deg']:.2f}°（单次验收）")
    ax[2].legend()
fig.suptitle("CrowerFOC USB低电流控制验收 · 2026-10-09", fontsize=15)
fig.savefig(OUT / "20261009_control_acceptance.png", dpi=160)
plt.close(fig)

fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
test_runs = segments(control["samples"], lambda x: x["state"] == 3 and x["mode"] == 2)
for i, (run, key, direction) in enumerate(zip(test_runs[:2], ("position_1_1", "position_1_-1"), ("+20°", "−20°"))):
    t = np.array([x["time_s"] for x in run]); pos = np.array([x["pos_deg"] for x in run])
    rpm = np.array([x["vel_rpm"] for x in run]); iq = np.array([x["iq_A"] for x in run])
    target = control["checks"][key]["target_deg"]
    error = control["checks"][key]["error_deg"]
    axes[i, 0].plot(t, pos); axes[i, 0].axhline(target, color="tab:red", ls="--")
    axes[i, 0].set_ylabel("位置 (°)"); axes[i, 0].set_title(f"{direction} 位置保持，误差 {error:.2f}°")
    axes[i, 1].plot(t, rpm, color="tab:blue", label="rpm")
    axes[i, 1].set_ylabel("速度 (rpm)", color="tab:blue")
    current_axis = axes[i, 1].twinx()
    current_axis.plot(t, iq, color="tab:orange", label="Iq (A)")
    current_axis.set_ylabel("Iq (A)", color="tab:orange")
    axes[i, 1].set_title("速度与电流（双纵轴）")
    lines = axes[i, 1].get_lines() + current_axis.get_lines()
    axes[i, 1].legend(lines, [line.get_label() for line in lines])
for a in axes.flat:
    a.set_xlabel("相对时间 (s)")
fig.suptitle("位置环反向重复性回归 · 1 A运行限流 · 验收未通过", fontsize=15)
fig.savefig(OUT / "20261009_position_repeatability.png", dpi=160)
plt.close(fig)

with (OUT / "20261009_control_samples.csv").open("w", newline="", encoding="utf-8-sig") as f:
    fields = ("source", "time_s", "state", "mode", "inmode", "pos_deg", "vel_rpm",
              "ia_A", "ib_A", "ic_A", "id_A", "iq_A", "iq_target_A", "vbus", "fault")
    writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    for source, result in (("feature_acceptance", feature), ("position_regression", control)):
        for row in result["samples"]:
            writer.writerow({"source": source, **row})
print("saved", OUT / "20261009_control_acceptance.png")
print("saved", OUT / "20261009_position_repeatability.png")
print("saved", OUT / "20261009_control_samples.csv")
