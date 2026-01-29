from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

# --------------------------------------------------
# Isaac imports (must come AFTER SimulationApp)
# --------------------------------------------------
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.articulations import Articulation

from isaac_sim.controllers.dodo_rl_controller import DodoRLController


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # --------------------------------------------------
    # Load Dodo USD
    # --------------------------------------------------
    dodo_usd_path = (
        "/home/youri/TUM/2025w/dodo/assets/dodo/usd/dodo.usd"
    )

    robot_prim_path = "/World/Dodo"

    add_reference_to_stage(
        usd_path=dodo_usd_path,
        prim_path=robot_prim_path,
    )

    dodo = Articulation(
        prim_path=robot_prim_path,
        name="dodo",
    )
    world.scene.add(dodo)

    world.reset()

    # --------------------------------------------------
    # Controller
    # --------------------------------------------------
    controller = DodoRLController(
        robot=dodo,
        policy_name="stand",
        action_scale=0.5,
    )

    print("[DODO] RL controller started")

    # --------------------------------------------------
    # Main loop
    # --------------------------------------------------
    while simulation_app.is_running():
        world.step(render=True)

        if not world.is_playing():
            continue

        obs = controller.get_observation()
        action = controller.compute_action(obs)
        controller.apply_action(action)

    simulation_app.close()


if __name__ == "__main__":
    main()
