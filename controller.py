"""
Controller — IK-based pick-and-toss state machine.
Moves the arm: HOME → APPROACH → DESCEND → GRASP → LIFT → TOSS → DONE
"""

import numpy as np
import config


class PickPlaceController:

    PHASE_NAMES = [
        "HOME", "APPROACH", "DESCEND", "GRASP", "LIFT", "SWING", "TOSS", "DONE",
    ]

    HOME_JOINTS = [0.0, 0.4, 0.0, -1.2, 0.0, 1.0, 0.0]

    def __init__(self, plant, target_pos):
        self.plant = plant
        self.phase = 0
        self.settle_counter  = 0
        self._timeout_counter = 0
        self.target_joints   = None
        self._bin_pos        = list(target_pos)
        self.waypoints       = self._build_waypoints(target_pos)

    def _build_waypoints(self, target_pos):
        px = config.BELT_PICK_X
        py = config.BELT_Y
        pz = config.BELT_PICK_Z
        # Swing waypoint: 65 % of the way to the bin, arc height above lift
        bx, by = target_pos[0], target_pos[1]
        sx = px + (bx - px) * 0.65
        sy = py + (by - py) * 0.65
        return [
            None,                                              # 0: HOME
            [px, py, pz + config.APPROACH_ABOVE_OBJECT],      # 1: APPROACH
            [px, py, pz + config.HOVER_ABOVE_OBJECT],         # 2: DESCEND
            None,                                              # 3: GRASP
            [px, py, config.LIFT_HEIGHT],                      # 4: LIFT
            [sx, sy, config.LIFT_HEIGHT + 0.15],               # 5: SWING
            None,                                              # 6: TOSS (action)
        ]

    def update(self):
        if self.phase >= len(self.PHASE_NAMES) - 1:
            return "DONE"

        phase_name = self.PHASE_NAMES[self.phase]

        if phase_name == "HOME":
            self.target_joints = np.array(self.HOME_JOINTS)
            self.plant.set_joint_targets(self.target_joints)
            actual, _ = self.plant.get_joint_states()
            if np.linalg.norm(actual - self.target_joints) < 0.05:
                self.settle_counter += 1
                if self.settle_counter >= config.GRASP_SETTLE_STEPS:
                    self._advance_phase()
            else:
                self.settle_counter = 0
            return phase_name

        if phase_name == "GRASP":
            self.settle_counter += 1
            if self.settle_counter >= config.GRASP_SETTLE_STEPS:
                if not self.plant.grasp_object():
                    print("[CTRL] Grasp failed — aborting.")
                    self._abort()
                else:
                    self._advance_phase()
            return phase_name

        if phase_name == "TOSS":
            # Kept as a fallback — should not normally be reached since SWING
            # now fires toss mid-motion. If we land here anyway, toss immediately.
            self.plant.toss_object(self._bin_pos)
            self._advance_phase()
            return phase_name

        # Movement phases (APPROACH, DESCEND, LIFT, SWING)
        waypoint = self.waypoints[self.phase]
        if waypoint is not None:
            joints = self.plant.compute_ik(waypoint,
                                           constrain_orn=(phase_name != "SWING"))
            if joints is None:
                print(f"[CTRL] IK degenerate in {phase_name} — aborting.")
                self._abort()
                return phase_name
            self.target_joints = joints
            self.plant.set_joint_targets(self.target_joints)

            ee_pos, _ = self.plant.get_end_effector_state()
            error = np.linalg.norm(ee_pos - np.array(waypoint))

            if phase_name == "SWING":
                # Release mid-swing: toss once the arm has covered ~30 % of the
                # swing arc so the object is released while the arm has momentum.
                swing_start = np.array(self.waypoints[self.phase - 1])  # LIFT wp
                total_dist  = np.linalg.norm(np.array(waypoint) - swing_start)
                progress    = 1.0 - (error / max(total_dist, 1e-6))
                if progress >= 0.55 or self._timeout_counter >= config.GRASP_SETTLE_STEPS * 8:
                    self.plant.toss_object(self._bin_pos)
                    self.phase          = len(self.PHASE_NAMES) - 1
                    self.settle_counter = 0
                    self._timeout_counter = 0
                    print(f"[CTRL] Phase → DONE  (mid-swing toss, progress={progress:.2f})")
                else:
                    self._timeout_counter += 1
            else:
                if error < config.IK_TOLERANCE * 3:
                    self.settle_counter += 1
                    if self.settle_counter >= config.PHASE_SETTLE_STEPS:
                        self._advance_phase()
                else:
                    self.settle_counter = 0
                    self._timeout_counter += 1
                    if self._timeout_counter >= config.GRASP_SETTLE_STEPS * 12:
                        print(f"[CTRL] Timeout in {phase_name}, error={error:.3f}m — forcing advance")
                        self._advance_phase()
                        self._timeout_counter = 0

        return phase_name

    def _advance_phase(self):
        self.phase += 1
        self.settle_counter   = 0
        self._timeout_counter = 0
        if self.phase < len(self.PHASE_NAMES):
            print(f"[CTRL] Phase → {self.PHASE_NAMES[self.phase]}")

    def _abort(self):
        self.plant.release_object()
        self.phase            = len(self.PHASE_NAMES) - 1
        self.settle_counter   = 0
        self._timeout_counter = 0

    def update_pick_pos(self, pick_pos):
        """Update approach/descend waypoints with the object's actual belt position."""
        px, py, pz = pick_pos[0], pick_pos[1], pick_pos[2]
        self.waypoints[1] = [px, py, pz + config.APPROACH_ABOVE_OBJECT]
        self.waypoints[2] = [px, py, pz + config.HOVER_ABOVE_OBJECT]
        self.waypoints[4] = [px, py, config.LIFT_HEIGHT]

    def reset(self, target_pos):
        """Start next task — skip HOME, go direct to approach."""
        self.phase            = 1
        self.settle_counter   = 0
        self._timeout_counter = 0
        self.target_joints    = None
        self._bin_pos         = list(target_pos)
        self.waypoints        = self._build_waypoints(target_pos)

    def is_done(self):
        return self.phase >= len(self.PHASE_NAMES) - 1

    def get_phase_name(self):
        return self.PHASE_NAMES[self.phase] if self.phase < len(self.PHASE_NAMES) else "DONE"

    def get_target_joints(self):
        return self.target_joints
