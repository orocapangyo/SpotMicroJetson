from math import cos, sin, sqrt, atan2, acos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matlab_leg import LegVisualizer

class EllipticalTrajectory:
    def __init__(self, center, a, b, height=50):
        """
        center : tuple (x, y, z)
            타원의 중심점
        a : float
            장축 반지름 (전후 방향)
        b : float
            단축 반지름 (좌우 방향)
        height : float
            발을 들어올리는 최대 높이
        """
        self.center = np.array(center)
        self.a = a
        self.b = b
        self.height = height

    def generate_step(self, phase):
        """
        한 걸음의 궤적 생성

        Parameters:
        -----------
        phase : float
            0.0 ~ 1.0 사이의 값 (0: 시작, 1: 끝)

        Returns:
        --------
        x, y, z : float
            발끝 위치
        """
        # 0.0 ~ 0.5: Swing phase (발을 들어올림)
        # 0.5 ~ 1.0: Stance phase (발이 지면에 닿음)

        if phase < 0.5:
            # Swing phase - 타원 궤적
            swing_phase = phase * 2  # 0 ~ 1로 정규화
            angle = np.pi * swing_phase

            x = self.center[0] + self.a * np.cos(angle)
            y = self.center[1]
            z = self.center[2] + self.height * np.sin(angle)
        else:
            # Stance phase - 직선 복귀
            stance_phase = (phase - 0.5) * 2  # 0 ~ 1로 정규화

            x = self.center[0] - self.a + 2 * self.a * stance_phase
            y = self.center[1]
            z = self.center[2]

        return x, y, z

    def plot_step(self):
        """
        한 걸음 궤적 시각화
        """
        phases = np.linspace(0, 1, 100)
        trajectory = np.array([self.generate_step(p) for p in phases])

        fig = plt.figure(figsize=(12, 5))

        # 측면도 (X-Z)
        ax1 = fig.add_subplot(121)

        # Swing phase
        swing_idx = 50
        ax1.plot(trajectory[:swing_idx, 0], trajectory[:swing_idx, 2],
                'b-', linewidth=3, label='Swing (발 들림)')

        # Stance phase
        ax1.plot(trajectory[swing_idx:, 0], trajectory[swing_idx:, 2],
                'r-', linewidth=3, label='Stance (발 닿음)')

        ax1.scatter(*self.center[[0, 2]], color='green', s=100, marker='x', label='Center')
        ax1.set_xlabel('X (mm)')
        ax1.set_ylabel('Z (mm)')
        ax1.grid(True)
        ax1.legend()
        ax1.set_title('Side View - One Step')
        ax1.axis('equal')

        # 3D 뷰
        ax2 = fig.add_subplot(122, projection='3d')
        ax2.plot3D(trajectory[:swing_idx, 0], trajectory[:swing_idx, 1], trajectory[:swing_idx, 2],
                  'b-', linewidth=3, label='Swing')
        ax2.plot3D(trajectory[swing_idx:, 0], trajectory[swing_idx:, 1], trajectory[swing_idx:, 2],
                  'r-', linewidth=3, label='Stance')
        ax2.set_xlabel('X (mm)')
        ax2.set_ylabel('Y (mm)')
        ax2.set_zlabel('Z (mm)')
        ax2.legend()
        ax2.set_title('3D View')

        plt.tight_layout()
        plt.show()
if __name__=="__main__":
    # 사용 예시
    ellipse_traj = EllipticalTrajectory(center=(100, -100, -100), a=40, b=20, height=60)
    ellipse_traj.plot_step()