from math import cos, sin, sqrt, atan2, acos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matlab_4leg_v import QuadrupedVisualizer
from matlab_kinematics import LegIK
# 클래스 생성
class QuadrupedController:
    def __init__(self):
        self.leg_ik = LegIK()
        self.quad_viz = QuadrupedVisualizer()

    def set_foot_positions(self, foot_positions):
        """
        4개 발의 목표 위치로부터 관절 각도 계산

        Parameters:
        -----------
        foot_positions : dict
            {'front_left': [x, y, z], 'front_right': [x, y, z], ...}
            (몸통 기준 로컬 좌표)

        Returns:
        --------
        leg_angles : dict
            {'front_left': [θ1, θ2, θ3], ...}
        """
        leg_angles = {}

        for leg_name, target_pos in foot_positions.items():
            # 각 다리의 IK 계산
            theta1, theta2, theta3 = self.leg_ik.inverse_kinematics(*target_pos)
            leg_angles[leg_name] = [theta1, theta2, theta3]

        return leg_angles

    def visualize_stance(self, foot_positions):
        """
        주어진 발 위치로 로봇 자세 시각화
        """
        # IK 계산
        leg_angles = self.set_foot_positions(foot_positions)

        # 시각화
        fig = plt.figure(figsize=(14, 10))
        ax = fig.add_subplot(111, projection='3d')

        self.quad_viz.draw_robot(ax, leg_angles)

        # 목표 발 위치 표시
        for leg_name, pos in foot_positions.items():
            base_pos = self.quad_viz.leg_positions[leg_name]
            global_pos = np.array(pos) + base_pos
            ax.scatter(*global_pos, color='red', s=50, marker='x')

        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_xlim([-200, 200])
        ax.set_ylim([-200, 200])
        ax.set_zlim([-300, 100])
        ax.legend()
        plt.show()

if __name__=="__main__":
    # 사용 예시 - Home Stance
    controller = QuadrupedController()

    # 4개 발의 목표 위치 (몸통 기준 로컬 좌표)
    home_stance = {
        'front_left':  [50, 100, -150],
        'front_right': [50, -100, -150],
        'rear_left':   [-50, 100, -150],
        'rear_right':  [-50, -100, -150]
    }

    #앉은 자세
    sitting_stance = {
    'front_left':  [80, 100, -100],   # 앞다리는 높게
    'front_right': [80, -100, -100],
    'rear_left':   [-50, 100, -200],  # 뒷다리는 낮게
    'rear_right':  [-50, -100, -200]
    }

    # 실험 2: 한쪽으로 기울기
    tilted_stance = {
    'front_left':  [50, 100, -120],   # 왼쪽 다리는 짧게
    'front_right': [50, -100, -180],  # 오른쪽 다리는 길게
    'rear_left':   [-50, 100, -120],
    'rear_right':  [-50, -100, -180]
    }
    
    # 실험 3: 한 발 들기
    tripod_stance = {
    'front_left':  [50, 100, -80],    # 왼쪽 앞발 들어올림
    'front_right': [50, -100, -150],
    'rear_left':   [-50, 100, -150],
    'rear_right':  [-50, -100, -150]
    }   

    controller.visualize_stance(tilted_stance)