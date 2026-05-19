"""
Configuration for Digital Twin simulation.
"""

# ─── Simulation ───
SIM_TIMESTEP = 1.0 / 240.0
GRAVITY      = -9.81
GUI_DELAY    = 1.0 / 60.0

# ─── Kuka Arm ───
KUKA_END_EFFECTOR_INDEX = 6
KUKA_NUM_JOINTS         = 7
KUKA_MAX_FORCE          = 800.0
KUKA_POSITION_GAIN      = 0.1
KUKA_VELOCITY_GAIN      = 1.0

# ─── Object types ───
OBJECT_TYPES = [
    "cube", "flat_box", "tall_box",       # box family    → bin 0
    "cylinder", "fat_cylinder",           # cylinder family → bin 1
    "sphere", "small_sphere",             # sphere family  → bin 2
]
OBJECT_SIZE = 0.04
OBJECT_COLORS = {
    "cube":         [0.85, 0.15, 0.15, 1.0],
    "flat_box":     [0.90, 0.55, 0.10, 1.0],
    "tall_box":     [0.65, 0.15, 0.80, 1.0],
    "cylinder":     [0.15, 0.35, 0.90, 1.0],
    "fat_cylinder": [0.10, 0.65, 0.80, 1.0],
    "sphere":       [0.15, 0.75, 0.25, 1.0],
    "small_sphere": [0.05, 0.85, 0.85, 1.0],
}
OBJECT_BIN = {
    "cube": 0, "flat_box": 0, "tall_box": 0,
    "cylinder": 1, "fat_cylinder": 1,
    "sphere": 2, "small_sphere": 2,
}

# ─── Bins ───
# Three bins in three corners; belt occupies the +X / near-zero-Y corner.
BIN_POSITIONS = [
    [ 0.0,  1.20, 0.0],   # bin 0 — BOXES      (red)    far front
    [-1.20,  0.0, 0.0],   # bin 1 — CYLINDERS  (blue)   far left
    [ 0.0, -1.20, 0.0],   # bin 2 — SPHERES    (green)  far back
]
BIN_COLORS = [
    [0.85, 0.20, 0.20, 0.65],
    [0.15, 0.35, 0.85, 0.65],
    [0.15, 0.75, 0.25, 0.65],
]
BIN_LABELS     = ["BOXES", "CYLINDERS", "SPHERES"]
BIN_INNER_HALF = 0.22    # much wider mouth for catching tossed objects
BIN_WALL_T     = 0.015
BIN_WALL_H     = 0.10    # taller walls to catch objects
BIN_BASE_H     = 0.008

# ─── Toss parameters ───
TOSS_TIME          = 0.45  # projectile flight time in seconds (physics)
TOSS_SETTLE_STEPS  = 25    # steps to hold at lift before releasing
ARC_STEPS_PER_TICK = 5     # arc positions advanced per main-loop step (visual speed)

# ─── Conveyor belt ───
# Belt runs along the X axis at fixed Y; objects travel from far-X toward arm
BELT_Y          =  0.00          # belt centre Y
BELT_START_X    =  1.00          # belt near end (arm side)
BELT_SPAWN_X    =  2.20          # far end where objects appear
BELT_END_X      =  0.45          # physical end of belt geometry
BELT_PICK_X     =  0.45          # X where belt stops; arm picks from here
BELT_TOP_Z      =  0.30          # top surface of belt (lowered for arm reach)
BELT_HALF_W     =  0.40          # belt half-width
BELT_HALF_H     =  0.025         # belt half-thickness
BELT_SPEED        = 0.009        # metres moved per simulation step
BELT_STRIPE_GAP   = 0.12         # gap between animated stripes (m)
BELT_NUM_STRIPES  = 14
BELT_STRIPE_EVERY = 1           # redraw stripes every N steps (reduces lag)

# Spawn Y range (objects appear anywhere across the belt width)
BELT_SPAWN_Y_MARGIN = 0.04       # keep objects this far from belt edge
# Pick Z (fixed); pick X and Y vary per object — computed dynamically
BELT_PICK_Z     = BELT_TOP_Z + OBJECT_SIZE

# ─── Multi-object batch & pick zone ───
PICK_ZONE_LENGTH   = 0.35        # how far back from BELT_PICK_X the zone extends (m)
BELT_SPAWN_MIN = 5              # min steps between object spawns
BELT_SPAWN_MAX = 20              # max steps between object spawns

# ─── Task heights ───
HOVER_ABOVE_OBJECT    = 0.07
APPROACH_ABOVE_OBJECT = 0.14   # reduced — belt is high, keep EE within arm reach
LIFT_HEIGHT           = 0.62  # must clear belt top (0.30) with margin

# ─── Controller ───
IK_TOLERANCE         = 0.02
IK_MAX_ITERATIONS    = 200
PHASE_SETTLE_STEPS   = 2
GRASP_SETTLE_STEPS   = 10
RELEASE_SETTLE_STEPS = 10


# ─── Camera ───
CAMERA_DISTANCE = 2.4
CAMERA_YAW      = 40
CAMERA_PITCH    = -35
CAMERA_TARGET   = [0.0, 0.0, 0.2]
