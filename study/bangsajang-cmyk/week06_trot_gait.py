import math
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class TrottingGait:
    """SpotMicro의 간단한 제자리 Trot 보행 궤적을 생성한다."""

    def __init__(self) -> None:
        # 단위: millisecond
        self.t0 = 300
        self.t1 = 1200
        self.t2 = 300
        self.t3 = 200

        # 발을 드는 높이
        self.step_height = 30.0

        # 기본 발 위치
        self.default_positions = {
            "FL": [100.0, -150.0, 80.0],
            "FR": [100.0, -150.0, -80.0],
            "RL": [-100.0, -150.0, 80.0],
            "RR": [-100.0, -150.0, -80.0],
        }

    @property
    def total_period(self) -> float:
        return self.t0 + self.t1 + self.t2 + self.t3

    def calculate_leg_position(
        self,
        elapsed_ms: float,
        initial_position: list[float],
    ) -> list[float]:
        """한 다리의 현재 발끝 위치를 계산한다."""
        x, y, z = initial_position
        elapsed_ms %= self.total_period

        # 1단계: 잠시 대기
        if elapsed_ms < self.t0:
            return [x, y, z]

        # 2단계: 발이 지면에 닿은 상태로 뒤쪽 이동
        if elapsed_ms < self.t0 + self.t1:
            progress = (elapsed_ms - self.t0) / self.t1
            x_offset = 40.0 - 80.0 * progress
            return [x + x_offset, y, z]

        # 3단계: 뒤쪽에서 잠시 대기
        if elapsed_ms < self.t0 + self.t1 + self.t2:
            return [x - 40.0, y, z]

        # 4단계: 발을 들어 앞쪽으로 되돌림
        swing_time = elapsed_ms - self.t0 - self.t1 - self.t2
        progress = swing_time / self.t3

        x_offset = -40.0 + 80.0 * progress
        y_offset = self.step_height * math.sin(math.pi * progress)

        return [x + x_offset, y + y_offset, z]

    def positions(self, current_time: float) -> dict[str, list[float]]:
        """대각선 다리에 반 주기 위상차를 적용한다."""
        elapsed_ms = current_time * 1000.0
        half_period = self.total_period / 2.0

        phase_a = elapsed_ms % self.total_period
        phase_b = (elapsed_ms + half_period) % self.total_period

        return {
            "FL": self.calculate_leg_position(
                phase_a, self.default_positions["FL"]
            ),
            "RR": self.calculate_leg_position(
                phase_a, self.default_positions["RR"]
            ),
            "FR": self.calculate_leg_position(
                phase_b, self.default_positions["FR"]
            ),
            "RL": self.calculate_leg_position(
                phase_b, self.default_positions["RL"]
            ),
        }


gait = TrottingGait()

figure, axis = plt.subplots()
axis.set_xlim(-160, 160)
axis.set_ylim(-190, -90)
axis.set_xlabel("X position")
axis.set_ylabel("Y position")
axis.set_title("Week06 - In-place Trot Gait")

points = {
    "FL": axis.plot([], [], "o", label="FL")[0],
    "FR": axis.plot([], [], "o", label="FR")[0],
    "RL": axis.plot([], [], "o", label="RL")[0],
    "RR": axis.plot([], [], "o", label="RR")[0],
}

axis.legend()


def update(_: int):
    current_time = time.time()
    positions = gait.positions(current_time)

    for leg_name, point in points.items():
        x, y, _ = positions[leg_name]
        point.set_data([x], [y])

    return list(points.values())


animation = FuncAnimation(
    figure,
    update,
    interval=20,
    blit=True,
    cache_frame_data=False,
)

plt.show()