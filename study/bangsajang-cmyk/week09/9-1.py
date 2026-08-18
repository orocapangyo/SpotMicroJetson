import gymnasium as gym
from gymnasium import spaces
import numpy as np


class SpotMicroEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # 12개 관절 제어값
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32
        )

        # 관측값 30개
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(30,),
            dtype=np.float32
        )

        self.step_count = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        self.step_count = 0

        observation = np.zeros(30, dtype=np.float32)

        return observation, {}

    def step(self, action):
        self.step_count += 1

        observation = np.random.randn(30).astype(np.float32)

        reward = 1.0

        terminated = False

        truncated = self.step_count >= 100

        return observation, reward, terminated, truncated, {}


if __name__ == "__main__":

    env = SpotMicroEnv()

    observation, info = env.reset()

    print("Observation Space :", env.observation_space)
    print("Action Space      :", env.action_space)
    print("Observation Shape :", observation.shape)

    for i in range(100):

        action = env.action_space.sample()

        observation, reward, terminated, truncated, info = env.step(action)

        print(
            f"Step {i + 1:03d} | "
            f"Reward: {reward:.1f}"
        )

        if terminated or truncated:
            break

    env.close()

    print("Week09 9-1 test completed.")