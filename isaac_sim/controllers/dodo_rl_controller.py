import torch
import numpy as np
from pathlib import Path

from omni.isaac.core.articulations import Articulation


class DodoRLController:
    """
    RL inference controller for Dodo robot (Isaac Sim only).
    - Loads trained policy
    - Converts observations to actions
    - Applies joint position targets
    """

    JOINT_NAMES = [
        "right_joint_1",
        "right_joint_2",
        "right_joint_3",
        "right_joint_4",
        "left_joint_1",
        "left_joint_2",
        "left_joint_3",
        "left_joint_4",
    ]

    def __init__(
        self,
        robot: Articulation,
        policy_name: str = "stand",
        action_scale: float = 0.5,
    ):
        self.robot = robot
        self.policy_name = policy_name
        self.action_scale = action_scale

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Resolve joint indices once
        self.joint_indices = [
            self.robot.get_dof_index(joint)
            for joint in self.JOINT_NAMES
        ]

        assert all(i >= 0 for i in self.joint_indices), \
            f"Invalid joint name in {self.JOINT_NAMES}"

        self.policy = self._load_policy(policy_name)

    # -----------------------------------------------------
    # Policy
    # -----------------------------------------------------
    def _load_policy(self, mode: str):
        policy_path = (
            Path(__file__).resolve()
            .parents[2]          # repo root
            / "models"
            / "policies"
            / mode
            / "policy.pt"
        )

        if not policy_path.exists():
            raise FileNotFoundError(f"Policy not found: {policy_path}")

        policy = torch.jit.load(policy_path, map_location=self.device)
        policy.eval()

        print(f"[DODO] Loaded policy: {mode}")
        return policy

    def switch_policy(self, mode: str):
        if mode == self.policy_name:
            return
        self.policy = self._load_policy(mode)
        self.policy_name = mode

    # -----------------------------------------------------
    # Observation
    # -----------------------------------------------------
    def get_observation(self) -> np.ndarray:
        """
        Observation vector layout (example, must match training):
        [ joint_pos (8),
          joint_vel (8) ]
        """
        joint_pos = self.robot.get_joint_positions()[self.joint_indices]
        joint_vel = self.robot.get_joint_velocities()[self.joint_indices]

        obs = np.concatenate([joint_pos, joint_vel], axis=0)
        return obs.astype(np.float32)

    # -----------------------------------------------------
    # Action
    # -----------------------------------------------------
    def compute_action(self, obs: np.ndarray) -> np.ndarray:
        obs_tensor = torch.from_numpy(obs).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.policy(obs_tensor)

        action = action.squeeze(0).cpu().numpy()
        return action

    def apply_action(self, action: np.ndarray):
        """
        Policy output assumed in [-1, 1]
        -> scaled delta position
        """
        current_pos = self.robot.get_joint_positions()[self.joint_indices]

        target_pos = current_pos + self.action_scale * action
        self.robot.set_joint_position_targets(
            target_pos,
            joint_indices=self.joint_indices
        )
