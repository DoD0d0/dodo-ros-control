import torch
import torch.nn as nn
import os

OBS_DIM = 48
ACT_DIM = 12

class DummyPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(OBS_DIM, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, ACT_DIM),
        )

    def forward(self, obs):
        return self.net(obs)


def main():
    model = DummyPolicy()
    model.eval()

    dummy_obs = torch.zeros(1, OBS_DIM)

    out_dir = os.path.join(os.path.dirname(__file__), "..", "models")
    os.makedirs(out_dir, exist_ok=True)

    out_path = os.path.join(out_dir, "policy.onnx")

    torch.onnx.export(
        model,
        dummy_obs,
        out_path,
        input_names=["obs"],
        output_names=["action"],
        opset_version=11,
    )

    print(f"[OK] Dummy ONNX exported to: {out_path}")
    print(f"     obs_dim={OBS_DIM}, act_dim={ACT_DIM}")


if __name__ == "__main__":
    main()