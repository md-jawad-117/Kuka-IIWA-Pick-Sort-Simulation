"""
Main — Kuka IIWA Pick & Sort simulation.

Usage:
    python main.py                  # GUI mode
    python main.py --headless       # No GUI
    python main.py --speed 3        # Run faster
    python main.py --max-steps 2000 # Stop after N steps
"""

import argparse
import time
import numpy as np

import config
from physical_plant import PhysicalPlant
from controller import PickPlaceController
from state_logger import StateLogger
from dashboard import Dashboard


def parse_args():
    parser = argparse.ArgumentParser(
        description="Kuka IIWA Pick & Sort Simulation"
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--max-steps", type=int, default=0)
    return parser.parse_args()


def run_simulation(args):
    print("=" * 60)
    print("  Kuka IIWA Pick & Sort Simulation")
    print("=" * 60)
    print(f"  GUI:       {'OFF' if args.headless else 'ON'}")
    print(f"  Max steps: {'unlimited' if args.max_steps == 0 else args.max_steps}")
    print("=" * 60)

    use_gui = not args.headless
    plant   = PhysicalPlant(use_gui=use_gui)
    logger  = StateLogger()

    gui_delay = config.GUI_DELAY / args.speed if use_gui else 0

    # ── STOP button ──
    import pybullet as _pb
    stop_button  = None
    stop_presses = 0
    if use_gui:
        stop_button = _pb.addUserDebugParameter(
            "STOP SIMULATION", 1, 0, 1,
            physicsClientId=plant.physics_client)
        stop_presses = _pb.readUserDebugParameter(
            stop_button, physicsClientId=plant.physics_client)

    def _stop_requested():
        nonlocal stop_presses
        if stop_button is None:
            return False
        val = _pb.readUserDebugParameter(stop_button, physicsClientId=plant.physics_client)
        if val > stop_presses:
            stop_presses = val
            return True
        return False

    # ── State ──
    belt_running = True
    pick_queue   = []
    controller   = None
    spawn_timer  = 0
    task_num     = 0

    def _start_next_pick():
        nonlocal controller, task_num
        obj        = pick_queue[0]
        bin_i      = config.OBJECT_BIN[obj['type']]
        target_pos = config.BIN_POSITIONS[bin_i]
        plant.set_current_object(obj['id'], obj['type'])
        task_num += 1
        print(f"[PICK] Task {task_num}: {obj['type']} → {config.BIN_LABELS[bin_i]}"
              f"  ({len(pick_queue)} in zone)")
        if controller is None:
            controller = PickPlaceController(plant, target_pos)
        else:
            controller.reset(target_pos)
        controller.update_pick_pos(plant.belt_pick_pos)
        if use_gui:
            plant.highlight_target_bin(bin_i)

    print("\n[SIM] Starting — objects spawning continuously...\n")

    step = 0
    try:
        while True:
            # ── 1. Spawn ──
            if belt_running:
                if spawn_timer <= 0:
                    plant.spawn_single_on_belt()
                    spawn_timer = np.random.randint(config.BELT_SPAWN_MIN,
                                                    config.BELT_SPAWN_MAX + 1)
                else:
                    spawn_timer -= 1

            # ── 2. Belt tick ──
            belt_just_stopped = plant.conveyor_tick()

            # ── 3. Belt stopped — queue pick zone objects ──
            if belt_running and belt_just_stopped:
                belt_running = False
                pick_queue   = plant.get_objects_in_zone()
                n_behind     = len(plant._objects) - len(pick_queue)
                print(f"\n[BELT] Stopped — {len(pick_queue)} in zone, "
                      f"{n_behind} still coming")
                if pick_queue:
                    _start_next_pick()
                else:
                    plant.restart_belt()
                    belt_running = True

            # ── 4. Arm picking ──
            if not belt_running and controller is not None:
                phase = controller.update()

                if controller.is_done():
                    plant.remove_current_object()
                    pick_queue.pop(0)

                    if pick_queue:
                        _start_next_pick()
                    else:
                        controller  = None
                        spawn_timer = 0
                        print("[BELT] Zone clear — belt restarting")
                        plant.restart_belt()
                        belt_running = True
            else:
                if belt_running:
                    # Return arm to home while belt is running so it's ready
                    plant.set_joint_targets(
                        np.array(PickPlaceController.HOME_JOINTS))
                phase = "CONVEYING" if belt_running else "WAITING"

            # ── 5. Physics + projectile arcs ──
            plant.step()
            plant.tick_projectiles()

            # ── 6. Log ──
            joint_pos, _ = plant.get_joint_states()
            ee_pos, _    = plant.get_end_effector_state()
            logger.log(step, phase, joint_pos, ee_pos)

            # ── 7. GUI updates ──
            if use_gui:
                plant.update_phase_label(phase)

            # ── 7. Progress print ──
            if step % 200 == 0:
                print(f"  Step {step:5d} | Task {task_num} | Phase: {phase:10s} | "
                      f"Belt: {'RUN' if belt_running else 'STP'} | "
                      f"Objects: {len(plant._objects)}")

            # ── 8. GUI delay ──
            if use_gui and gui_delay > 0:
                time.sleep(gui_delay)

            # ── 9. Stop conditions ──
            if _stop_requested():
                print(f"\n[SIM] STOP pressed at step {step}.")
                break
            if args.max_steps > 0 and step >= args.max_steps:
                break

            step += 1

    except KeyboardInterrupt:
        print(f"\n[SIM] Ctrl-C at step {step}.")

    print("\n" + "=" * 60)
    print("  SIMULATION COMPLETE")
    print("=" * 60)
    logger.save()
    print("\n[DASH] Generating plots...")
    Dashboard.plot_results(logger)
    Dashboard.plot_3d_trajectory(logger)
    plant.disconnect()
    print("\nDone.")


if __name__ == "__main__":
    args = parse_args()
    run_simulation(args)
