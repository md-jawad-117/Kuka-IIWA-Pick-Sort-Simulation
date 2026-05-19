"""
State Logger — records joint states, EE position and phase to CSV.
"""

import csv
import config


class StateLogger:

    def __init__(self, filename="simulation_log.csv"):
        self.filename = filename
        self.records  = []
        self.headers  = (
            ["step", "phase"]
            + [f"joint_{i}" for i in range(config.KUKA_NUM_JOINTS)]
            + ["ee_x", "ee_y", "ee_z"]
        )

    def log(self, step, phase, joint_pos, ee_position):
        record = [step, phase] + list(joint_pos) + list(ee_position)
        self.records.append(record)

    def save(self):
        with open(self.filename, "w", newline="") as f:
            csv.writer(f).writerows([self.headers] + self.records)
        print(f"[LOG] Saved {len(self.records)} records → {self.filename}")
