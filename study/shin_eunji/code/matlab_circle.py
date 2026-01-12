from math import cos, sin, sqrt, atan2, acos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matlab_leg import LegVisualizer

class TrajectoryGenerator:
    def __init__(self, center, radius, height_offset=0):
        """
        center : tuple (x, y, z)
            원의 중심점
        radius : float
            원의 반지름
        height_offset : float
            수직 방향 오프셋
        """
        self.center = np.array(center)
        self.radius = radius
        self.height_offset = height_offset

    def circular_trajectory(self, t, period=2.0):
        """
        원형 궤적 생성

        Parameters:
        -----------
        t : float
            시간 (초)
        period : float
            한 바퀴 도는 시간 (초)

        Returns:
        --------
        x, y, z : float
            시간 t에서의 발끝 위치
        """
        angle = 2 * np.pi * t / period

        x = self.center[0] + self.radius * np.cos(angle)
        y = self.center[1] + self.radius * np.sin(angle)
        z = self.center[2] + self.height_offset * np.sin(2 * angle)

        return x, y, z

    def plot_trajectory(self, duration=2.0, dt=0.01):
        """
        궤적을 3D로 시각화
        """
        times = np.arange(0, duration, dt)
        trajectory = np.array([self.circular_trajectory(t) for t in times])

        fig = plt.figure(figsize=(12, 5))

        # 3D 뷰
        ax1 = fig.add_subplot(121, projection='3d')
        ax1.plot3D(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
                  'b-', linewidth=2, label='Trajectory')
        ax1.scatter(*self.center, color='red', s=100, marker='x', label='Center')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Y (mm)')
        ax1.set_zlabel('Z (mm)')
        ax1.legend()
        ax1.set_title('3D Trajectory')

        # 2D 투영 (XY 평면)
        ax2 = fig.add_subplot(122)
        ax2.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2)
        ax2.scatter(self.center[0], self.center[1], color='red', s=100, marker='x')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.grid(True)
        ax2.axis('equal')
        ax2.set_title('Top View (XY Plane)')

        plt.tight_layout()
        plt.show()
if __name__=="__main__":
    # 사용 예시
    traj_gen = TrajectoryGenerator(center=(100, -100, -100), radius=20, height_offset=20)
    traj_gen.plot_trajectory(duration=2.0)