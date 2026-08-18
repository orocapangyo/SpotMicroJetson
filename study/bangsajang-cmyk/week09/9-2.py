import argparse
import os

import gymnasium as gym
from gymnasium import spaces
import numpy as np

from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv


# =========================================================
# SpotMicro Gymnasium Environment
# =========================================================
class SpotMicroEnv(gym.Env):

    def __init__(self):
        super().__init__()

        # 12개 관절 제어
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32
        )

        # 관측값 30차원
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(30,),
            dtype=np.float32
        )

        self.step_count = 0
        self.max_steps = 100

    def reset(self, seed=None, options=None):

        super().reset(seed=seed)

        self.step_count = 0

        observation = np.zeros(
            30,
            dtype=np.float32
        )

        info = {}

        return observation, info

    def step(self, action):

        self.step_count += 1

        # 테스트용 관측값
        observation = np.random.randn(
            30
        ).astype(np.float32)

        # 기본 보상
        reward = 1.0

        # 행동이 너무 크면 약간 감점
        reward -= 0.01 * float(
            np.sum(np.square(action))
        )

        terminated = False

        truncated = (
            self.step_count >= self.max_steps
        )

        info = {}

        return (
            observation,
            reward,
            terminated,
            truncated,
            info
        )


# =========================================================
# Environment 생성
# =========================================================
def make_env():

    def _init():

        env = SpotMicroEnv()

        env = Monitor(env)

        return env

    return _init


# =========================================================
# PPO Training
# =========================================================
def train():

    os.makedirs(
        "models",
        exist_ok=True
    )

    os.makedirs(
        "logs",
        exist_ok=True
    )

    env = DummyVecEnv(
        [make_env()]
    )

    model = PPO(
        policy="MlpPolicy",
        env=env,

        learning_rate=3e-4,

        n_steps=2048,

        batch_size=64,

        n_epochs=10,

        gamma=0.99,

        gae_lambda=0.95,

        clip_range=0.2,

        ent_coef=0.01,

        verbose=1,

        tensorboard_log="./logs/ppo_spotmicro/"
    )

    print("=" * 50)
    print("Week09 PPO Training Started")
    print("=" * 50)

    # 과제 요구: 100,000 steps
    model.learn(
        total_timesteps=100_000,
        progress_bar=False
    )

    model.save(
        "./models/ppo_spotmicro_final"
    )

    env.close()

    print("=" * 50)
    print("Training complete!")
    print(
        "Model saved:"
        " ./models/ppo_spotmicro_final.zip"
    )
    print("=" * 50)


# =========================================================
# Evaluation
# =========================================================
def evaluate(model_path):

    env = DummyVecEnv(
        [make_env()]
    )

    model = PPO.load(
        model_path,
        env=env
    )

    obs = env.reset()

    total_reward = 0.0

    print("=" * 50)
    print("PPO Evaluation Started")
    print("=" * 50)

    for step in range(100):

        action, _states = model.predict(
            obs,
            deterministic=True
        )

        obs, rewards, dones, infos = env.step(
            action
        )

        reward = float(rewards[0])

        total_reward += reward

        print(
            f"Step {step + 1:03d}"
            f" | Reward: {reward:.3f}"
        )

        if dones[0]:

            obs = env.reset()

    print("=" * 50)

    print(
        f"Total Reward: "
        f"{total_reward:.3f}"
    )

    print(
        f"Average Reward: "
        f"{total_reward / 100:.3f}"
    )

    print("=" * 50)

    env.close()


# =========================================================
# Main
# =========================================================
if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--train",
        action="store_true",
        help="Train PPO model"
    )

    parser.add_argument(
        "--eval",
        type=str,
        help="Evaluate trained PPO model"
    )

    args = parser.parse_args()

    if args.train:

        train()

    elif args.eval:

        evaluate(
            args.eval
        )

    else:

        print(
            "Usage:"
        )

        print(
            "python 9-2.py --train"
        )

        print(
            "python 9-2.py "
            "--eval ./models/ppo_spotmicro_final"
        )