import numpy as np
import onnxruntime as ort
from pathlib import Path


def main():
    # --------------------------------------------------
    # Path setup
    # --------------------------------------------------
    model_path = Path(__file__).parent.parent / "models" / "policy.onnx"

    if not model_path.exists():
        raise FileNotFoundError(
            f"ONNX model not found: {model_path}\n"
            "Place your exported policy at mujoco/models/policy.onnx"
        )

    # --------------------------------------------------
    # Load ONNX model
    # --------------------------------------------------
    session = ort.InferenceSession(
        model_path.as_posix(),
        providers=["CPUExecutionProvider"],
    )

    print("Loaded ONNX model")
    print("Inputs :", session.get_inputs())
    print("Outputs:", session.get_outputs())

    # --------------------------------------------------
    # Dummy observation (CHANGE THIS LATER)
    # --------------------------------------------------
    obs_dim = session.get_inputs()[0].shape[-1]
    dummy_obs = np.zeros((1, obs_dim), dtype=np.float32)

    # --------------------------------------------------
    # Run inference
    # --------------------------------------------------
    input_name = session.get_inputs()[0].name
    outputs = session.run(None, {input_name: dummy_obs})

    action = outputs[0]

    print("Inference OK")
    print("Action shape:", action.shape)
    print("Action:", action)


if __name__ == "__main__":
    main()