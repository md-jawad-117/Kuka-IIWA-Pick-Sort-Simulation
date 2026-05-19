"""
Physical Plant — robot arm + conveyor belt simulation.
"""

import pybullet as p
import pybullet_data
import numpy as np
import config


class PhysicalPlant:

    def __init__(self, use_gui=True):
        self.use_gui = use_gui
        mode = p.GUI if use_gui else p.DIRECT
        self.physics_client = p.connect(mode)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, config.GRAVITY, physicsClientId=self.physics_client)
        p.setTimeStep(config.SIM_TIMESTEP, physicsClientId=self.physics_client)

        if use_gui:
            p.resetDebugVisualizerCamera(
                cameraDistance=config.CAMERA_DISTANCE,
                cameraYaw=config.CAMERA_YAW,
                cameraPitch=config.CAMERA_PITCH,
                cameraTargetPosition=config.CAMERA_TARGET,
                physicsClientId=self.physics_client,
            )

        self.plane_id = p.loadURDF("plane.urdf", physicsClientId=self.physics_client)
        self.kuka_id  = p.loadURDF(
            "kuka_iiwa/model.urdf", basePosition=[0, 0, 0],
            useFixedBase=True, physicsClientId=self.physics_client,
        )

        self._create_conveyor()
        self._create_bins()

        # Multi-object belt state
        # Each entry: {'id': int, 'type': str, 'x': float, 'y': float}
        self._objects        = []
        self._belt_stopped   = False

        # Kinematic arc animations for in-flight projectiles
        # Each entry: {'id': int, 'positions': [...], 'index': int}
        self._projectiles    = []

        # Currently targeted/grasped object (set per pick cycle)
        self.object_id           = None
        self.current_object_type = None
        self.grasp_constraint    = None

        self._belt_offset = 0.0
        self.joint_indices = list(range(config.KUKA_NUM_JOINTS))

        # Live UI state
        self._phase_label_id = None
        self._obj_label_id   = None
        self._active_bin_idx = -1
        self._highlight_ids  = []

        if use_gui:
            self._create_pedestal()
            self._create_safety_fence()
            self._create_belt_rollers()
            self._create_floor_markings()
            self._create_bin_base_plates()
            self._create_pick_zone_glow()
            self._init_phase_label()

    # ── Conveyor belt ─────────────────────────────────────────────────────────

    def _create_conveyor(self):
        cx       = (config.BELT_SPAWN_X + config.BELT_END_X) / 2
        cy       = config.BELT_Y
        half_len = (config.BELT_SPAWN_X - config.BELT_END_X) / 2
        belt_centre_z = config.BELT_TOP_Z - config.BELT_HALF_H

        col = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[half_len, config.BELT_HALF_W, config.BELT_HALF_H],
            physicsClientId=self.physics_client)
        vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[half_len, config.BELT_HALF_W, config.BELT_HALF_H],
            rgbaColor=[0.18, 0.18, 0.18, 1.0],
            physicsClientId=self.physics_client)
        p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=[cx, cy, belt_centre_z],
            physicsClientId=self.physics_client)

        # Side rails
        for side in [-1, 1]:
            ry = cy + side * (config.BELT_HALF_W + 0.006)
            col2 = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=[half_len, 0.005, config.BELT_HALF_H + 0.008],
                physicsClientId=self.physics_client)
            vis2 = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[half_len, 0.005, config.BELT_HALF_H + 0.008],
                rgbaColor=[0.55, 0.55, 0.55, 1.0],
                physicsClientId=self.physics_client)
            p.createMultiBody(
                baseMass=0, baseCollisionShapeIndex=col2, baseVisualShapeIndex=vis2,
                basePosition=[cx, ry, belt_centre_z],
                physicsClientId=self.physics_client)

        # Support legs
        belt_bottom_z = belt_centre_z - config.BELT_HALF_H
        leg_half_h    = max(belt_bottom_z / 2.0, 0.005)
        leg_xs = [config.BELT_END_X + 0.15,
                  (config.BELT_SPAWN_X + config.BELT_END_X) / 2,
                  config.BELT_SPAWN_X - 0.15]
        for lx in leg_xs:
            for side in [-1, 1]:
                ly = cy + side * (config.BELT_HALF_W - 0.03)
                col3 = p.createCollisionShape(
                    p.GEOM_BOX, halfExtents=[0.014, 0.014, leg_half_h],
                    physicsClientId=self.physics_client)
                vis3 = p.createVisualShape(
                    p.GEOM_BOX, halfExtents=[0.014, 0.014, leg_half_h],
                    rgbaColor=[0.40, 0.40, 0.40, 1.0],
                    physicsClientId=self.physics_client)
                p.createMultiBody(
                    baseMass=0, baseCollisionShapeIndex=col3, baseVisualShapeIndex=vis3,
                    basePosition=[lx, ly, leg_half_h],
                    physicsClientId=self.physics_client)

        # Pick-zone front edge marker
        y0 = config.BELT_Y - config.BELT_HALF_W
        y1 = config.BELT_Y + config.BELT_HALF_W
        z  = config.BELT_TOP_Z + 0.003
        p.addUserDebugLine(
            [config.BELT_PICK_X, y0, z], [config.BELT_PICK_X, y1, z],
            lineColorRGB=[1, 0.9, 0], lineWidth=3,
            physicsClientId=self.physics_client)

        self._stripe_tick = 0
        self._stripe_ids  = self._init_stripes() if self.use_gui else []

    def _init_stripes(self):
        y0 = config.BELT_Y - config.BELT_HALF_W + 0.01
        y1 = config.BELT_Y + config.BELT_HALF_W - 0.01
        z  = config.BELT_TOP_Z + 0.003
        ids = []
        for _ in range(config.BELT_NUM_STRIPES):
            lid = p.addUserDebugLine(
                [config.BELT_SPAWN_X, y0, z],
                [config.BELT_SPAWN_X, y1, z],
                lineColorRGB=[0.45, 0.45, 0.45], lineWidth=1.5,
                physicsClientId=self.physics_client)
            ids.append(lid)
        return ids

    def _update_belt_stripes(self):
        self._stripe_tick += 1
        if self._stripe_tick % config.BELT_STRIPE_EVERY != 0:
            return
        if not self.use_gui or not self._stripe_ids:
            return
        y0       = config.BELT_Y - config.BELT_HALF_W + 0.01
        y1       = config.BELT_Y + config.BELT_HALF_W - 0.01
        z        = config.BELT_TOP_Z + 0.003
        belt_len = config.BELT_SPAWN_X - config.BELT_END_X
        period   = belt_len + config.BELT_STRIPE_GAP
        for i, lid in enumerate(self._stripe_ids):
            x = config.BELT_SPAWN_X - (i * config.BELT_STRIPE_GAP + self._belt_offset) % period
            p.addUserDebugLine(
                [x, y0, z], [x, y1, z],
                lineColorRGB=[0.45, 0.45, 0.45], lineWidth=1.5,
                replaceItemUniqueId=lid,
                physicsClientId=self.physics_client)

    # ── Bins ──────────────────────────────────────────────────────────────────

    def _create_bins(self):
        for i, label in enumerate(config.BIN_LABELS):
            bpos  = config.BIN_POSITIONS[i]
            color = config.BIN_COLORS[i]
            self._build_bin(bpos[0], bpos[1], color)
            p.addUserDebugText(
                label, [bpos[0], bpos[1], 0.20],
                textColorRGB=[0, 0, 0], textSize=1.2,
                physicsClientId=self.physics_client)

    def _build_bin(self, cx, cy, color):
        s = config.BIN_INNER_HALF
        t = config.BIN_WALL_T
        h = config.BIN_WALL_H
        b = config.BIN_BASE_H
        parts = [
            ([cx,       cy,       b    ], [s+t, s+t, b]),
            ([cx-(s+t), cy,       b*2+h], [t,   s+t, h]),
            ([cx+(s+t), cy,       b*2+h], [t,   s+t, h]),
            ([cx,       cy-(s+t), b*2+h], [s+t, t,   h]),
            ([cx,       cy+(s+t), b*2+h], [s+t, t,   h]),
        ]
        for pos, half in parts:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color,
                                      physicsClientId=self.physics_client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                              baseVisualShapeIndex=vis, basePosition=pos,
                              physicsClientId=self.physics_client)

    # ── Environment dressing ──────────────────────────────────────────────────

    def _create_pedestal(self):
        for half, z, color in [
            ([0.14, 0.14, 0.03], 0.03, [0.25, 0.25, 0.28, 1.0]),
            ([0.10, 0.10, 0.02], 0.08, [0.30, 0.30, 0.33, 1.0]),
        ]:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color,
                                      physicsClientId=self.physics_client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                              baseVisualShapeIndex=vis, basePosition=[0, 0, z],
                              physicsClientId=self.physics_client)

    def _create_safety_fence(self):
        r        = 0.95
        hw       = 0.008
        hh       = 0.055
        col_rgba = [0.95, 0.80, 0.05, 0.85]
        segments = [
            (-r,  0.0, hw,      r + hw),
            ( 0.0,  r, r + hw,  hw    ),
            ( 0.0, -r, r + hw,  hw    ),
        ]
        for cx, cy, hx, hy in segments:
            c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hh],
                                       physicsClientId=self.physics_client)
            v = p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hh],
                                    rgbaColor=col_rgba,
                                    physicsClientId=self.physics_client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c,
                              baseVisualShapeIndex=v,
                              basePosition=[cx, cy, hh],
                              physicsClientId=self.physics_client)
        for sy in [-1, 1]:
            c = p.createCollisionShape(p.GEOM_BOX, halfExtents=[hw*2, hw*2, hh*1.5],
                                       physicsClientId=self.physics_client)
            v = p.createVisualShape(p.GEOM_BOX, halfExtents=[hw*2, hw*2, hh*1.5],
                                    rgbaColor=[0.95, 0.80, 0.05, 1.0],
                                    physicsClientId=self.physics_client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=c,
                              baseVisualShapeIndex=v,
                              basePosition=[r, sy * r, hh * 1.5],
                              physicsClientId=self.physics_client)

    def _create_belt_rollers(self):
        orn        = p.getQuaternionFromEuler([np.pi / 2, 0, 0])
        belt_cz    = config.BELT_TOP_Z - config.BELT_HALF_H
        roller_r   = config.BELT_HALF_H + 0.004
        roller_len = config.BELT_HALF_W * 2 + 0.025
        col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=roller_r, height=roller_len,
            physicsClientId=self.physics_client)
        vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=roller_r, length=roller_len,
            rgbaColor=[0.50, 0.50, 0.52, 1.0],
            physicsClientId=self.physics_client)
        p.createMultiBody(
            baseMass=0, baseCollisionShapeIndex=col, baseVisualShapeIndex=vis,
            basePosition=[config.BELT_SPAWN_X, config.BELT_Y, belt_cz],
            baseOrientation=orn,
            physicsClientId=self.physics_client)

    def _create_floor_markings(self):
        grid_color = [0.28, 0.28, 0.28]
        z = 0.001
        for x in np.arange(-1.5, 3.3, 0.5):
            p.addUserDebugLine([x, -2.2, z], [x, 2.2, z],
                lineColorRGB=grid_color, lineWidth=0.5,
                physicsClientId=self.physics_client)
        for y in np.arange(-2.2, 2.3, 0.5):
            p.addUserDebugLine([-1.5, y, z], [3.2, y, z],
                lineColorRGB=grid_color, lineWidth=0.5,
                physicsClientId=self.physics_client)

    def _create_bin_base_plates(self):
        pad = 0.03
        s   = config.BIN_INNER_HALF + config.BIN_WALL_T + pad
        for i, bpos in enumerate(config.BIN_POSITIONS):
            r, g, b, _ = config.BIN_COLORS[i]
            plate_color = [r * 0.7, g * 0.7, b * 0.7, 1.0]
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s, s, 0.004],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s, s, 0.004],
                                      rgbaColor=plate_color,
                                      physicsClientId=self.physics_client)
            p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                              baseVisualShapeIndex=vis,
                              basePosition=[bpos[0], bpos[1], 0.004],
                              physicsClientId=self.physics_client)

    def _create_pick_zone_glow(self):
        """Rectangle showing the full pick zone on the belt surface."""
        y0 = config.BELT_Y - config.BELT_HALF_W + 0.01
        y1 = config.BELT_Y + config.BELT_HALF_W - 0.01
        x0 = config.BELT_PICK_X
        x1 = config.BELT_PICK_X + config.PICK_ZONE_LENGTH
        z  = config.BELT_TOP_Z + 0.005
        corners = [[x0,y0,z],[x1,y0,z],[x1,y1,z],[x0,y1,z]]
        for i in range(4):
            p.addUserDebugLine(corners[i], corners[(i+1)%4],
                lineColorRGB=[1, 1, 0.3], lineWidth=2,
                physicsClientId=self.physics_client)
        # Label inside zone
        p.addUserDebugText(
            "PICK ZONE",
            [x0 + (x1-x0)*0.5, config.BELT_Y, z + 0.04],
            textColorRGB=[1, 1, 0.4], textSize=0.8,
            physicsClientId=self.physics_client)

    def _init_phase_label(self):
        self._phase_label_id = p.addUserDebugText(
            "IDLE", [0.0, 0.0, 1.05],
            textColorRGB=[1.0, 1.0, 0.2], textSize=1.4,
            physicsClientId=self.physics_client)

    # ── Live UI updates ───────────────────────────────────────────────────────

    def update_phase_label(self, phase_name):
        if not self.use_gui or self._phase_label_id is None:
            return
        colors = {
            "CONVEYING": [0.6, 0.6, 0.6],
            "WAITING":   [0.9, 0.9, 0.3],
            "HOME":      [0.8, 0.8, 0.8],
            "APPROACH":  [0.3, 0.8, 1.0],
            "DESCEND":   [0.3, 0.8, 1.0],
            "GRASP":     [1.0, 0.6, 0.1],
            "LIFT":      [0.4, 1.0, 0.4],
            "MOVE":      [0.4, 1.0, 0.4],
            "DESCEND2":  [1.0, 0.5, 0.5],
            "RELEASE":   [1.0, 0.3, 0.3],
            "TOSS":      [1.0, 0.8, 0.0],
            "RETREAT":   [0.6, 0.6, 1.0],
            "DONE":      [0.5, 1.0, 0.5],
        }
        color = colors.get(phase_name, [1.0, 1.0, 0.2])
        self._phase_label_id = p.addUserDebugText(
            phase_name, [0.0, 0.0, 1.05],
            textColorRGB=color, textSize=1.4,
            replaceItemUniqueId=self._phase_label_id,
            physicsClientId=self.physics_client)

    def highlight_target_bin(self, bin_idx):
        if not self.use_gui or bin_idx == self._active_bin_idx:
            return
        for lid in self._highlight_ids:
            p.removeUserDebugItem(lid, physicsClientId=self.physics_client)
        self._highlight_ids  = []
        self._active_bin_idx = bin_idx
        bpos = config.BIN_POSITIONS[bin_idx]
        r, g, b, _ = config.BIN_COLORS[bin_idx]
        s = config.BIN_INNER_HALF + config.BIN_WALL_T + 0.04
        z = 0.006
        corners = [
            [bpos[0]-s, bpos[1]-s, z], [bpos[0]+s, bpos[1]-s, z],
            [bpos[0]+s, bpos[1]+s, z], [bpos[0]-s, bpos[1]+s, z],
        ]
        for i in range(4):
            lid = p.addUserDebugLine(
                corners[i], corners[(i+1)%4],
                lineColorRGB=[r, g, b], lineWidth=4,
                physicsClientId=self.physics_client)
            self._highlight_ids.append(lid)

    # ── Multi-object belt logic ───────────────────────────────────────────────

    def spawn_single_on_belt(self):
        """Spawn one random object at the far end in a random lane."""
        obj_type = np.random.choice(config.OBJECT_TYPES)
        y_min = config.BELT_Y - config.BELT_HALF_W + config.BELT_SPAWN_Y_MARGIN
        y_max = config.BELT_Y + config.BELT_HALF_W - config.BELT_SPAWN_Y_MARGIN
        y     = float(np.random.uniform(y_min, y_max))
        obj_id = self._create_object(obj_type, config.BELT_SPAWN_X, y)
        self._objects.append({'id': obj_id, 'type': obj_type,
                              'x': config.BELT_SPAWN_X, 'y': y})
        print(f"[BELT] → {obj_type} at y={y:.2f}")

    def _create_object(self, obj_type, x, y):
        """Create one object body and return its id."""
        color = config.OBJECT_COLORS[obj_type]
        s     = config.OBJECT_SIZE
        pos   = [x, y, config.BELT_PICK_Z]

        if obj_type == "cube":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s, s, s],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s, s, s],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "flat_box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s*1.2, s*1.2, s*0.5],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s*1.2, s*1.2, s*0.5],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "tall_box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s*0.6, s*0.6, s*1.0],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s*0.6, s*0.6, s*1.0],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "cylinder":
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=s, height=s*2,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=s, length=s*2,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "fat_cylinder":
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=s*1.4, height=s,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=s*1.4, length=s,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "sphere":
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=s,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=s,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        else:  # small_sphere
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=s*0.75,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=s*0.75,
                                      rgbaColor=color, physicsClientId=self.physics_client)

        return p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=pos,
            physicsClientId=self.physics_client,
        )

    def _spawn_kinematic_copy(self, obj_type, position):
        """Spawn a massless (kinematic) visual copy of obj_type at position."""
        return self._create_projectile(obj_type, position)

    def _create_projectile(self, obj_type, position):
        """Spawn a kinematic visual body whose position is driven by the arc animator."""
        color = config.OBJECT_COLORS[obj_type]
        s     = config.OBJECT_SIZE

        if obj_type == "cube":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s, s, s],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s, s, s],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "flat_box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s*1.2, s*1.2, s*0.5],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s*1.2, s*1.2, s*0.5],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "tall_box":
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s*0.6, s*0.6, s*1.0],
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[s*0.6, s*0.6, s*1.0],
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "cylinder":
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=s, height=s*2,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=s, length=s*2,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "fat_cylinder":
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=s*1.4, height=s,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=s*1.4, length=s,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        elif obj_type == "sphere":
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=s,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=s,
                                      rgbaColor=color, physicsClientId=self.physics_client)
        else:  # small_sphere
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=s*0.75,
                                         physicsClientId=self.physics_client)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=s*0.75,
                                      rgbaColor=color, physicsClientId=self.physics_client)

        proj_id = p.createMultiBody(
            baseMass=0.15,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=position,
            physicsClientId=self.physics_client,
        )
        return proj_id

    def conveyor_tick(self):
        """
        Advance belt animation and move all free objects.
        Returns True when belt is (or just became) stopped.
        """
        if not self._belt_stopped:
            self._belt_offset = (self._belt_offset + config.BELT_SPEED) % config.BELT_STRIPE_GAP
        self._update_belt_stripes()

        if self._belt_stopped:
            return True

        triggered = False
        for obj in self._objects:
            if self.grasp_constraint is not None and obj['id'] == self.object_id:
                continue  # grasped — don't slide it
            if obj['x'] > config.BELT_PICK_X:
                obj['x'] = max(obj['x'] - config.BELT_SPEED, config.BELT_PICK_X)
                p.resetBasePositionAndOrientation(
                    obj['id'],
                    [obj['x'], obj['y'], config.BELT_PICK_Z],
                    [0, 0, 0, 1],
                    physicsClientId=self.physics_client)
            if obj['x'] <= config.BELT_PICK_X:
                triggered = True

        if triggered:
            self._belt_stopped = True
        return triggered

    def restart_belt(self):
        """Allow belt to move again after zone has been cleared."""
        self._belt_stopped = False

    def get_objects_in_zone(self):
        """Return list of object dicts whose X is within the pick zone."""
        x_near = config.BELT_PICK_X - 0.02   # small tolerance
        x_far  = config.BELT_PICK_X + config.PICK_ZONE_LENGTH
        return [o for o in self._objects if x_near <= o['x'] <= x_far]

    def has_objects_on_belt(self):
        return len(self._objects) > 0

    def set_current_object(self, obj_id, obj_type):
        """Point the arm at a specific object for the next pick cycle."""
        self.object_id           = obj_id
        self.current_object_type = obj_type

    @property
    def belt_pick_pos(self):
        """Pick position of the currently targeted object."""
        for o in self._objects:
            if o['id'] == self.object_id:
                return [o['x'], o['y'], config.BELT_PICK_Z]
        return [config.BELT_PICK_X, config.BELT_Y, config.BELT_PICK_Z]

    def toss_object(self, target_pos):
        """Throw object to target_pos using projectile motion.

        Animate the projectile kinematically along a pre-computed parabolic arc
        so that the visual flight speed is independent of the simulation timestep.
        The arc advances config.ARC_STEPS_PER_TICK positions per main-loop step,
        making the throw appear much faster than the raw physics rate.
        """
        if self.object_id is None:
            return
        oid      = self.object_id
        obj_type = self.current_object_type

        # Capture the object's world position
        obj_pos, _ = p.getBasePositionAndOrientation(
            oid, physicsClientId=self.physics_client)
        x0, y0, z0 = obj_pos

        # Projectile velocity needed to reach the bin from current position
        T  = config.TOSS_TIME
        dx = target_pos[0] - x0
        dy = target_pos[1] - y0
        dz = float(target_pos[2]) - z0
        vx_tgt = dx / T
        vy_tgt = dy / T
        vz_tgt = (dz + 0.5 * 9.81 * T * T) / T

        # Pure projectile math — shorter TOSS_TIME keeps vz low so the arc
        # stays flat and natural without a jarring upward jerk at release.
        vx = vx_tgt
        vy = vy_tgt
        vz = vz_tgt

        # Pre-compute arc: positions at SIM_TIMESTEP intervals
        dt = config.SIM_TIMESTEP
        N  = max(int(T * 2 / dt), 10)   # 2× T gives room to land
        positions = []
        for i in range(N):
            t = i * dt
            px = x0 + vx * t
            py = y0 + vy * t
            pz = z0 + vz * t - 0.5 * 9.81 * t * t
            if pz < 0.01:
                positions.append([px, py, 0.01])
                break
            positions.append([px, py, pz])

        # Spawn kinematic stand-in BEFORE touching the original body
        proj_id = self._spawn_kinematic_copy(obj_type, list(obj_pos))

        # Release constraint then hide the original body underground —
        # avoids the physics-engine rebuild that p.removeBody() causes.
        self.release_object()
        p.resetBasePositionAndOrientation(
            oid, [0, 0, -10], [0, 0, 0, 1],
            physicsClientId=self.physics_client)

        self._projectiles.append({'id': proj_id, 'oid': oid,
                                   'positions': positions, 'index': 0})

        print(f"[TOSS] arc {len(positions)} steps  "
              f"v=({vx:.2f},{vy:.2f},{vz:.2f}) m/s")

        self._objects            = [o for o in self._objects if o['id'] != oid]
        self.object_id           = None
        self.current_object_type = None

    def tick_projectiles(self):
        """Advance all in-flight arc animations. Call once per main-loop step."""
        finished = []
        for proj in self._projectiles:
            proj['index'] = min(
                proj['index'] + config.ARC_STEPS_PER_TICK,
                len(proj['positions']) - 1)
            pos = proj['positions'][proj['index']]
            p.resetBasePositionAndOrientation(
                proj['id'], pos, [0, 0, 0, 1],
                physicsClientId=self.physics_client)
            if proj['index'] >= len(proj['positions']) - 1:
                finished.append(proj)
        for proj in finished:
            self._projectiles.remove(proj)
            # Original body is already at z=-10 (underground) — leave it there.
            # removeBody() forces a physics broadphase rebuild causing a visible
            # frame hitch, so we never call it. Underground bodies are invisible
            # and don't interact with anything in the scene.

    def remove_current_object(self):
        """Hide the currently grasped/targeted object (move underground)."""
        self.release_object()
        if self.object_id is not None:
            p.resetBasePositionAndOrientation(
                self.object_id, [0, 0, -10], [0, 0, 0, 1],
                physicsClientId=self.physics_client)
            self._objects = [o for o in self._objects if o['id'] != self.object_id]
            self.object_id           = None
            self.current_object_type = None

    # ── Arm control ───────────────────────────────────────────────────────────

    def get_joint_states(self):
        states = p.getJointStates(self.kuka_id, self.joint_indices,
                                  physicsClientId=self.physics_client)
        return (np.array([s[0] for s in states]),
                np.array([s[1] for s in states]))

    def get_end_effector_state(self):
        state = p.getLinkState(self.kuka_id, config.KUKA_END_EFFECTOR_INDEX,
                               physicsClientId=self.physics_client)
        return np.array(state[0]), np.array(state[1])

    def set_joint_targets(self, targets):
        for i, jidx in enumerate(self.joint_indices):
            t = targets[i]
            p.setJointMotorControl2(
                self.kuka_id, jidx, p.POSITION_CONTROL,
                targetPosition=t, force=config.KUKA_MAX_FORCE,
                positionGain=config.KUKA_POSITION_GAIN,
                velocityGain=config.KUKA_VELOCITY_GAIN,
                physicsClientId=self.physics_client)

    _REST_POSES  = [0.0,  0.4, 0.0, -1.2, 0.0, 1.0, 0.0]
    _JOINT_LOWER = [-2.97, -2.09, -2.97, -2.09, -2.97, -1.57, -2.97]
    _JOINT_UPPER = [ 2.97,  2.09,  2.97,  2.09,  2.97,  1.57,  2.97]
    _JOINT_RANGE = [ 5.94,  4.18,  5.94,  4.18,  5.94,  3.14,  5.94]

    # Rest poses biased for a throwing/sweeping posture — shoulder rotates out,
    # elbow extends, wrist snaps forward for a natural throw arc.
    _THROW_POSES = [0.4, -0.8, 0.6, -0.2, 0.8, 1.2, 0.6]

    def compute_ik(self, target_pos, target_orn=None, constrain_orn=True):
        rest = self._REST_POSES if constrain_orn else self._THROW_POSES
        if constrain_orn:
            if target_orn is None:
                target_orn = p.getQuaternionFromEuler([0, -np.pi, 0])
            joints = p.calculateInverseKinematics(
                self.kuka_id, config.KUKA_END_EFFECTOR_INDEX,
                target_pos, target_orn,
                lowerLimits=self._JOINT_LOWER, upperLimits=self._JOINT_UPPER,
                jointRanges=self._JOINT_RANGE, restPoses=rest,
                maxNumIterations=config.IK_MAX_ITERATIONS,
                residualThreshold=config.IK_TOLERANCE,
                physicsClientId=self.physics_client)
        else:
            joints = p.calculateInverseKinematics(
                self.kuka_id, config.KUKA_END_EFFECTOR_INDEX,
                target_pos,
                lowerLimits=self._JOINT_LOWER, upperLimits=self._JOINT_UPPER,
                jointRanges=self._JOINT_RANGE, restPoses=rest,
                maxNumIterations=config.IK_MAX_ITERATIONS,
                residualThreshold=config.IK_TOLERANCE,
                physicsClientId=self.physics_client)
        joints = np.array(joints[: config.KUKA_NUM_JOINTS])
        if np.linalg.norm(np.array(target_pos)) > 0.25 and np.linalg.norm(joints) < 0.3:
            return None
        return joints

    def grasp_object(self):
        if self.grasp_constraint is not None:
            return True
        if self.object_id is None:
            return False
        ee_pos, _ = self.get_end_effector_state()
        obj_pos   = self.get_object_position()
        dist = np.linalg.norm(ee_pos - obj_pos)
        print(f"[GRASP] EE={ee_pos.round(3)}  OBJ={obj_pos.round(3)}  dist={dist:.3f}")
        if dist > 0.12:
            return False
        p.changeDynamics(self.object_id, -1, mass=0.15,
                         physicsClientId=self.physics_client)
        self.grasp_constraint = p.createConstraint(
            self.kuka_id, config.KUKA_END_EFFECTOR_INDEX,
            self.object_id, -1, p.JOINT_FIXED,
            [0, 0, 0], [0, 0, 0.05], [0, 0, 0],
            physicsClientId=self.physics_client)
        return True

    def release_object(self):
        if self.grasp_constraint is not None:
            p.removeConstraint(self.grasp_constraint,
                               physicsClientId=self.physics_client)
            self.grasp_constraint = None

    def get_object_position(self):
        if self.object_id is None:
            return np.zeros(3)
        pos, _ = p.getBasePositionAndOrientation(
            self.object_id, physicsClientId=self.physics_client)
        return np.array(pos)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _draw_ring(self, cx, cy, radius, color, z=0.002, segments=24):
        for i in range(segments):
            a0 = 2 * np.pi * i / segments
            a1 = 2 * np.pi * (i + 1) / segments
            p.addUserDebugLine(
                [cx + radius * np.cos(a0), cy + radius * np.sin(a0), z],
                [cx + radius * np.cos(a1), cy + radius * np.sin(a1), z],
                lineColorRGB=color, lineWidth=2,
                physicsClientId=self.physics_client)

    def step(self):
        p.stepSimulation(physicsClientId=self.physics_client)

    def disconnect(self):
        p.disconnect(physicsClientId=self.physics_client)
