"""
Dashboard — post-run plots: joint trajectories and 3-D EE trajectory.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import config

_PHASE_COLORS = [
    "#e8f4f8", "#fff3e0", "#e8f5e9", "#fce4ec",
    "#f3e5f5", "#e3f2fd", "#fff8e1", "#e0f2f1",
]


def _phase_spans(steps, phases):
    spans, cur_ph, cur_s = [], phases[0], steps[0]
    for s, ph in zip(steps[1:], phases[1:]):
        if ph != cur_ph:
            spans.append((cur_s, s, cur_ph))
            cur_ph, cur_s = ph, s
    spans.append((cur_s, steps[-1], cur_ph))
    return spans


def _draw_bands(ax, spans, y_lo, y_hi):
    for i, (x0, x1, ph) in enumerate(spans):
        ax.axvspan(x0, x1, alpha=0.22, color=_PHASE_COLORS[i % len(_PHASE_COLORS)], zorder=0)
        ax.text((x0+x1)/2, y_lo + 0.96*(y_hi-y_lo), ph,
                ha="center", va="top", fontsize=6, color="#444", rotation=45, clip_on=True)
        if i > 0:
            ax.axvline(x=x0, color="#999", linewidth=0.6, linestyle="--", alpha=0.5, zorder=1)


class Dashboard:

    @staticmethod
    def plot_results(logger, save_path="dashboard.png"):
        records = logger.records
        if not records:
            print("[DASH] No data.")
            return

        n = config.KUKA_NUM_JOINTS
        steps  = [r[0] for r in records]
        phases = [r[1] for r in records]
        joints = np.array([[r[2+j] for j in range(n)] for r in records])
        ee     = np.array([[r[2+n+k] for k in range(3)] for r in records])

        spans  = _phase_spans(steps, phases)
        colors = plt.cm.tab10(np.linspace(0, 1, n))

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(15, 8), sharex=True)
        fig.suptitle("Kuka IIWA Pick & Sort", fontsize=15, fontweight="bold")

        # ── Joint angles ──
        for j in range(n):
            ax1.plot(steps, joints[:, j], color=colors[j], linewidth=1.6, alpha=0.9)
        y_lo, y_hi = ax1.get_ylim()
        _draw_bands(ax1, spans, y_lo, y_hi)
        handles = [mpatches.Patch(color=colors[j], label=f"J{j}") for j in range(n)]
        ax1.legend(handles=handles, loc="upper right", fontsize=7, ncol=4)
        ax1.set_ylabel("Joint Angle (rad)")
        ax1.set_title("Joint Positions")
        ax1.grid(True, alpha=0.2)

        # ── EE trajectory ──
        ax2.plot(steps, ee[:, 0], label="X", color="tab:blue",   linewidth=1.5)
        ax2.plot(steps, ee[:, 1], label="Y", color="tab:orange",  linewidth=1.5)
        ax2.plot(steps, ee[:, 2], label="Z", color="tab:green",   linewidth=1.5)
        y_lo, y_hi = ax2.get_ylim()
        _draw_bands(ax2, spans, y_lo, y_hi)
        ax2.set_xlabel("Simulation Step")
        ax2.set_ylabel("Position (m)")
        ax2.set_title("End Effector Trajectory")
        ax2.legend(loc="upper right")
        ax2.grid(True, alpha=0.2)

        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[DASH] Saved → {save_path}")
        plt.show()

    @staticmethod
    def plot_3d_trajectory(logger, save_path="trajectory_3d.png"):
        records = logger.records
        if not records:
            return

        n      = config.KUKA_NUM_JOINTS
        ee     = np.array([[r[2+n+k] for k in range(3)] for r in records])
        phases = [r[1] for r in records]

        fig = plt.figure(figsize=(10, 8))
        ax  = fig.add_subplot(111, projection="3d")

        phase_list  = list(dict.fromkeys(phases))
        cmap        = plt.cm.tab10(np.linspace(0, 1, len(phase_list)))
        phase_color = {ph: cmap[i] for i, ph in enumerate(phase_list)}

        prev = 0
        for i in range(1, len(phases)):
            if phases[i] != phases[i-1] or i == len(phases)-1:
                seg = ee[prev:i+1]
                ax.plot(seg[:,0], seg[:,1], seg[:,2],
                        color=phase_color[phases[prev]], linewidth=1.8,
                        alpha=0.85, label=phases[prev])
                prev = i

        ax.scatter(*ee[0],  color="green",  s=120, label="Start", zorder=5)
        ax.scatter(*ee[-1], color="red",    s=120, label="End",   zorder=5)
        ax.scatter(config.BELT_PICK_X, config.BELT_Y, config.BELT_PICK_Z,
                   color="orange", s=100, marker="^", label="Pick zone", zorder=5)

        handles, labels = ax.get_legend_handles_labels()
        seen = {}
        for h, l in zip(handles, labels):
            if l not in seen:
                seen[l] = h
        ax.legend(seen.values(), seen.keys(), fontsize=8)

        ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
        ax.set_title("End Effector 3D Trajectory — coloured by phase")

        plt.savefig(save_path, dpi=150, bbox_inches="tight")
        print(f"[DASH] Saved → {save_path}")
        plt.show()
