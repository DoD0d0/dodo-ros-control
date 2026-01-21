import mujoco
import mujoco.viewer
from pathlib import Path
import time
import numpy as np

def main():
    urdf_path = Path(__file__).resolve().parents[1] / "assets/dodo/dodobot_v3/urdf/dodo.urdf"
    assert urdf_path.exists(), f"URDF not found: {urdf_path}"

    print(f"[INFO] Loading URDF: {urdf_path}")

    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    data = mujoco.MjData(model)

    print("[INFO] Model loaded")
    print(f"  nq = {model.nq}")
    print(f"  nv = {model.nv}")
    print(f"  nu = {model.nu}")
    print(f"  joints = {[model.joint(i).name for i in range(model.njnt)]}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        t = 0.0
        while viewer.is_running():
            # simple sinusoidal joint motion
            for i in range(model.nu):
                data.ctrl[i] = 0.3 * np.sin(t)

            mujoco.mj_step(model, data)
            viewer.sync()

            t += 0.01
            time.sleep(0.01)

if __name__ == "__main__":
    main()