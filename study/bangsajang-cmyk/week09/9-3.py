import argparse
import os

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv


class SpotMicroRewardEnv(gym.Env):

    def __init__(
        self,
        forward_vel_weight=2.0,
        energy_weight=-0.005,
        smoothness_weight=-0.1,
    ):
        super().__init__()

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32
        )

        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(30,),
            dtype=np.float32
        )

        self.forward_vel_weight = forward_vel_weight
        self.energy_weight = energy_weight
        self.smoothness_weight = smoothness_weight

        self.step_count = 0
        self.max_steps = 100

        self.previous_action = np.zeros(
            12,
            dtype=np.float32
        )

    def reset(self, seed=None, options=None):

        super().reset(seed=seed)

        self.step_count = 0

        self.previous_action = np.zeros(
            12,
            dtype=np.float32
        )

        observation = np.zeros(
            30,
            dtype=np.float32
        )

        return observation, {}

    def step(self, action):

        self.step_count += 1

        # -------------------------------
        # 가상의 로봇 상태
        # -------------------------------

        forward_velocity = float(
            np.mean(action)
        )

        energy_cost = float(
            np.sum(np.square(action))
        )

        smoothness_cost = float(
            np.sum(
                np.square(
                    action - self.previous_action
                )
            )
        )

        # -------------------------------
        # Reward Shaping
        # -------------------------------

        alive_reward = 1.0

        forward_reward = (
            self.forward_vel_weight
            * forward_velocity
        )

        energy_penalty = (
            self.energy_weight
            * energy_cost
        )

        smoothness_penalty = (
            self.smoothness_weight
            * smoothness_cost
        )

        reward = (
            alive_reward
            + forward_reward
            + energy_penalty
            + smoothness_penalty
        )

        self.previous_action = action.copy()

        observation = np.random.randn(
            30
        ).astype(np.float32)

        terminated = False

        truncated = (
            self.step_count >= self.max_steps
        )

        info = {
            "forward_velocity": forward_velocity,
            "energy_cost": energy_cost,
            "smoothness_cost": smoothness_cost
        }

        return (
            observation,
            reward,
            terminated,
            truncated,
            info
        )


def make_env(experiment):

    if experiment == "A":

        # 전진 보상 제거
        forward_vel_weight = 0.0
        energy_weight = -0.005
        smoothness_weight = -0.1

    elif experiment == "B":

        # 에너지 사용 패널티 증가
        forward_vel_weight = 2.0
        energy_weight = -0.1
        smoothness_weight = -0.1

    elif experiment == "C":

        # 급격한 움직임 패널티 증가
        forward_vel_weight = 2.0
        energy_weight = -0.005
        smoothness_weight = -0.5

    else:

        # 기본 설정
        forward_vel_weight = 2.0
        energy_weight = -0.005
        smoothness_weight = -0.1

    def _init():

        env = SpotMicroRewardEnv(
            forward_vel_weight=forward_vel_weight,
            energy_weight=energy_weight,
            smoothness_weight=smoothness_weight
        )

        return Monitor(env)

    return _init


def train(experiment):

    model_dir = (
        f"./models/week09_exp_{experiment}"
    )

    log_dir = (
        f"./logs/week09_exp_{experiment}"
    )

    os.makedirs(
        model_dir,
        exist_ok=True
    )

    os.makedirs(
        log_dir,
        exist_ok=True
    )

    env = DummyVecEnv(
        [
            make_env(
                experiment
            )
        ]
    )

    model = PPO(
        "MlpPolicy",
        env,

        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,

        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,

        verbose=1,

        tensorboard_log=log_dir
    )

    print("=" * 60)

    print(
        f"Reward Shaping Experiment {experiment}"
    )

    print("=" * 60)

    model.learn(
        total_timesteps=100_000
    )

    save_path = (
        f"{model_dir}/ppo_exp_{experiment}"
    )

    model.save(
        save_path
    )

    print("=" * 60)

    print(
        f"Experiment {experiment} complete!"
    )

    print(
        f"Model saved: {save_path}.zip"
    )

    print("=" * 60)

    env.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--experiment",
        choices=[
            "A",
            "B",
            "C"
        ],
        required=True
    )

    args = parser.parse_args()

    train(
        args.experiment
    )