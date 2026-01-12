
from math import cos, sin, sqrt, atan2, acos
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class LegIK:
    def __init__(self, l1=50, l2=20, l3=100, l4=100):
        """
        l1: Shoulder offset (mm)
        l2: Shoulder length (mm)
        l3: Upper leg length (mm)
        l4: Lower leg length (mm)
        """
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4

    def inverse_kinematics(self, x, y, z):
        """
        역기구학: 발끝 위치 (x, y, z) → 관절 각도 (θ1, θ2, θ3)

        Parameters:
        -----------
        x, y, z : float
            목표 발끝 위치 (mm)

        Returns:
        --------
        theta1, theta2, theta3 : float
            관절 각도 (radians)
        """
        l1, l2, l3, l4 = self.l1, self.l2, self.l3, self.l4

        # Step 1: Abduction 각도 (θ1)
        F = sqrt(x**2 + y**2 - l1**2)
        theta1 = -atan2(y, x) - atan2(F, -l1)

        # Step 2: Hip-Knee 평면으로 축소
        G = F - l2
        H = sqrt(G**2 + z**2)

        # 도달 가능성 확인
        if H > (l3 + l4) or H < abs(l3 - l4):
            raise ValueError(f"Target ({x}, {y}, {z}) is unreachable. H={H:.2f}")

        # Step 3: Knee 각도 (θ3) - Cosine Law
        cos_theta3 = (H**2 - l3**2 - l4**2) / (2 * l3 * l4)
        cos_theta3 = np.clip(cos_theta3, -1.0, 1.0)  # 수치 오차 방지
        theta3 = acos(cos_theta3)

        # Step 4: Hip 각도 (θ2)
        alpha = atan2(z, G)
        beta = atan2(l4 * sin(theta3), l3 + l4 * cos(theta3))
        theta2 = alpha - beta

        return theta1, theta2, theta3

    def forward_kinematics(self, theta1, theta2, theta3):
        """
        정기구학: 관절 각도 → 발끝 위치 (검증용)
        """
        l1, l2, l3, l4 = self.l1, self.l2, self.l3, self.l4
        theta23 = theta2 + theta3

        x = -l1*cos(theta1) - l2*sin(theta1) - l3*sin(theta1)*cos(theta2) - l4*sin(theta1)*cos(theta23)
        y = l1*sin(theta1) - l2*cos(theta1) - l3*cos(theta1)*cos(theta2) - l4*cos(theta1)*cos(theta23)
        z = l3*sin(theta2) + l4*sin(theta23)

        return x, y, z

# 검증 예시
if __name__=="__main__":
    leg_ik = LegIK()

    # 목표 발끝 위치
    target_pos = (100, -100, -150)

    # IK 계산
    theta1, theta2, theta3 = leg_ik.inverse_kinematics(*target_pos)
    print(f"관절 각도: θ1={np.rad2deg(theta1):.2f}°, θ2={np.rad2deg(theta2):.2f}°, θ3={np.rad2deg(theta3):.2f}°")

    # FK로 검증
    actual_pos = leg_ik.forward_kinematics(theta1, theta2, theta3)
    print(f"목표 위치: {target_pos}")
    print(f"실제 위치: ({actual_pos[0]:.2f}, {actual_pos[1]:.2f}, {actual_pos[2]:.2f})")
    print(f"오차: {np.linalg.norm(np.array(target_pos) - np.array(actual_pos)):.4f} mm")