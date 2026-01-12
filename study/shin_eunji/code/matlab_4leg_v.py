from math import cos, sin, sqrt, atan2, acos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matlab_leg import LegVisualizer

class QuadrupedVisualizer:
    def __init__(self):
        self.leg_viz = LegVisualizer()

        # 몸통 크기
        self.body_length = 140  # mm
        self.body_width = 75   # mm

        # 각 다리의 기준점 위치 (몸통 기준)
        self.leg_positions = {
            'front_left':  np.array([self.body_length/2, self.body_width/2, 0]),
            'front_right': np.array([self.body_length/2, -self.body_width/2, 0]),
            'rear_left':   np.array([-self.body_length/2, self.body_width/2, 0]),
            'rear_right':  np.array([-self.body_length/2, -self.body_width/2, 0])
        }

        self.colors = {
            'front_left': 'blue',
            'front_right': 'red',
            'rear_left': 'green',
            'rear_right': 'orange'
        }

    def draw_body(self, ax):
        """몸통 그리기"""
        L = self.body_length / 2
        W = self.body_width / 2

        # 몸통 4개 모서리
        corners = np.array([
            [L, W, 0], [L, -W, 0], [-L, -W, 0], [-L, W, 0], [L, W, 0]
        ])

        ax.plot3D(corners[:, 0], corners[:, 1], corners[:, 2],
                 color='black', linewidth=2)

    def draw_robot(self, ax, leg_angles):
        """
        전체 로봇 그리기
        leg_angles: 딕셔너리 {'front_left': [θ1, θ2, θ3], ...}
        """
        # 몸통 그리기
        self.draw_body(ax)

        # 각 다리 그리기
        for leg_name, base_pos in self.leg_positions.items():
            theta1, theta2, theta3 = leg_angles[leg_name]

            # 다리 관절 위치 계산
            points = self.leg_viz.calc_leg_points(theta1, theta2, theta3)

            # 몸통 기준점으로 이동
            points_global = points + base_pos

            # 그리기
            ax.plot3D(points_global[:, 0], points_global[:, 1], points_global[:, 2],
                     color=self.colors[leg_name], linewidth=2,
                     marker='o', markersize=4, label=leg_name)


# 사용 예시
if __name__=="__main__":
    quad_viz = QuadrupedVisualizer()

    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 모든 다리를 home position으로 설정
    leg_angles = {
        'front_left':  [0.0, -0.8, 1.6],
        'front_right': [0.0, -0.8, 1.6],
        'rear_left':   [0.0, -0.8, 1.6],
        'rear_right':  [0.0, -0.8, 1.6]
    }

    quad_viz.draw_robot(ax, leg_angles)

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_xlim([-200, 200])
    ax.set_ylim([-200, 200])
    ax.set_zlim([-300, 100])
    ax.legend()
    ax.set_title('SpotMicro - Home Position')

    plt.show()