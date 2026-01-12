import numpy as np
import math
import matplotlib.pyplot as plt

class FootTrajectory:
    """
    한 발의 발끝 궤적을 생성하는 클래스
    """
    def __init__(self, step_length=60, step_height=40, ground_height=-150):
        """
        Parameters:
        -----------
        step_length : float
            한 걸음의 보폭 (mm)
        step_height : float
            발을 들어올리는 높이 (mm)
        ground_height : float
            지면 높이 (몸체 기준, mm)
        """
        self.step_length = step_length
        self.step_height = step_height
        self.ground_height = ground_height

    def get_position(self, phase):
        """
        phase에 따른 발끝 위치 계산

        Parameters:
        -----------
        phase : float
            0.0 ~ 1.0 사이의 값
            0.0 ~ 0.5: Swing phase
            0.5 ~ 1.0: Stance phase

        Returns:
        --------
        x, z : float
            발끝의 X좌표(전후)와 Z좌표(상하)
        """
        Sl = self.step_length
        Sh = self.step_height
        ground_z = self.ground_height

        if phase < 0.5:
            # ===== Swing Phase =====
            # 반원 궤적으로 발을 앞으로 이동
            swing_phase = phase * 2  # 0 ~ 1로 정규화

            # X: 뒤에서 앞으로 이동
            x = -Sl/2 + Sl * swing_phase

            # Z: 반원 궤적 (sin 곡선)
            z = ground_z + Sh * math.sin(math.pi * swing_phase)
        else:
            # ===== Stance Phase =====
            # 지면에 닿아 직선으로 복귀
            stance_phase = (phase - 0.5) * 2  # 0 ~ 1로 정규화

            # X: 앞에서 뒤로 이동
            x = Sl/2 - Sl * stance_phase

            # Z: 지면 높이 유지
            z = ground_z

        return x, z
    
def visualize_foot_trajectory():
    """
    발끝 궤적을 시각화
    """
    traj = FootTrajectory(step_length=60, step_height=40, ground_height=-150)

    phases = np.linspace(0, 1, 100)
    positions = [traj.get_position(p) for p in phases]
    x_vals = [p[0] for p in positions]
    z_vals = [p[1] for p in positions]

    fig, ax = plt.subplots(figsize=(10, 6))

    # Swing phase (파란색)
    swing_idx = 50
    ax.plot(x_vals[:swing_idx], z_vals[:swing_idx],
            'b-', linewidth=3, label='Swing Phase (발 들림)')

    # Stance phase (빨간색)
    ax.plot(x_vals[swing_idx:], z_vals[swing_idx:],
            'r-', linewidth=3, label='Stance Phase (지면 접촉)')

    # 시작/끝 점 표시
    ax.scatter(x_vals[0], z_vals[0], color='green', s=100, zorder=5, label='Start')
    ax.scatter(x_vals[-1], z_vals[-1], color='orange', s=100, zorder=5, label='End')

    # 화살표로 방향 표시
    ax.annotate('', xy=(x_vals[25], z_vals[25]),
                xytext=(x_vals[20], z_vals[20]),
                arrowprops=dict(arrowstyle='->', color='blue', lw=2))

    ax.annotate('', xy=(x_vals[75], z_vals[75]),
                xytext=(x_vals[70], z_vals[70]),
                arrowprops=dict(arrowstyle='->', color='red', lw=2))

    ax.set_xlabel('X Position (mm)', fontsize=12)
    ax.set_ylabel('Z Position (mm)', fontsize=12)
    ax.set_title('Foot Trajectory - Swing/Stance Phase', fontsize=14)
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_aspect('equal')

    plt.tight_layout()
    plt.show()

# 실행
visualize_foot_trajectory()    