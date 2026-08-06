"""
Week07: SpotMicro 전진/후진, 좌우 회전 및 커브 주행 실습

실행 위치와 관계없이 SpotMicroJetson 프로젝트 루트를 자동으로 찾습니다.

키보드
------
W : 전진
S : 후진
A : 제자리 왼쪽 회전
D : 제자리 오른쪽 회전
Q : 전진 + 왼쪽 커브
E : 전진 + 오른쪽 커브
SPACE : 정지
R : 로봇 및 측정값 초기화
P : 현재 PyBullet 화면을 images/week07 폴더에 저장
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path
from typing import Final

import numpy as np
import pybullet as p


SCRIPT_PATH: Final[Path] = Path(__file__).resolve()
PROJECT_ROOT: Final[Path] = SCRIPT_PATH.parents[3]
SCREENSHOT_DIR: Final[Path] = SCRIPT_PATH.parent / "images"

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from Kinematics.kinematicMotion import TrottingGait
from Simulation.spotmicroai import Robot


MODE_COMMANDS: Final[dict[str, tuple[float, float]]] = {
    "STOP": (0.0, 0.0),
    "FORWARD": (1.0, 0.0),
    "BACKWARD": (-1.0, 0.0),
    "TURN_LEFT": (0.0, -1.0),
    "TURN_RIGHT": (0.0, 1.0),
    "CURVE_LEFT": (1.0, -1.0),
    "CURVE_RIGHT": (1.0, 1.0),
}

SCREENSHOT_FILES: Final[dict[str, str]] = {
    "FORWARD": "forward_walking.png",
    "BACKWARD": "parameter_test.png",
    "TURN_LEFT": "turning_walk.png",
    "TURN_RIGHT": "turning_walk.png",
    "CURVE_LEFT": "curve_walking.png",
    "CURVE_RIGHT": "curve_walking.png",
    "STOP": "parameter_test.png",
}


class Week07WalkingSimulation:
    """SpotMicro Week07 키보드 보행 실습 컨트롤러."""

    def __init__(self) -> None:
        SCREENSHOT_DIR.mkdir(parents=True, exist_ok=True)

        self.robot = Robot(
            useFixedBase=False,
            useStairs=False,
            resetFunc=None,
        )
        self.gait = TrottingGait()
        self.stand_positions = np.array(self.robot.Lp, dtype=float).copy()

        self.mode = "STOP"
        self.start_time = time.time()
        self.origin_position = np.array(self.robot.getPos(), dtype=float)
        self.status_text_id = -1

        # Week07 핵심 파라미터 조정용 슬라이더
        self.step_length_id = p.addUserDebugParameter(
            "Step Length Sl (mm)", 30.0, 120.0, 60.0
        )
        self.step_height_id = p.addUserDebugParameter(
            "Step Height Sh (mm)", 20.0, 80.0, 40.0
        )
        self.stance_time_id = p.addUserDebugParameter(
            "Stance Time t1 (ms)", 600.0, 1500.0, 1200.0
        )
        self.swing_time_id = p.addUserDebugParameter(
            "Swing Time t3 (ms)", 100.0, 400.0, 200.0
        )
        self.step_alpha_id = p.addUserDebugParameter(
            "Step Alpha Sa (deg)", 5.0, 20.0, 15.0
        )
        self.body_height_id = p.addUserDebugParameter(
            "Body Height Offset (mm)", -20.0, 50.0, 20.0
        )

        print("=" * 68)
        print("Week07 SpotMicro 키보드 보행 시뮬레이션")
        print("W/S: 전진·후진 | A/D: 제자리 회전 | Q/E: 커브 주행")
        print("SPACE: 정지 | R: 초기화 | P: 화면 저장 | Ctrl+C: 종료")
        print("=" * 68)

    @staticmethod
    def _key_triggered(keys: dict[int, int], key: str) -> bool:
        """문자 키가 새로 눌렸는지 확인합니다."""
        codes = {ord(key.lower()), ord(key.upper())}
        return any(
            keys.get(code, 0) & p.KEY_WAS_TRIGGERED
            for code in codes
        )

    def process_keyboard(self) -> None:
        """PyBullet 창에서 들어온 키 입력으로 주행 모드를 변경합니다."""
        keys = p.getKeyboardEvents()

        key_to_mode = {
            "w": "FORWARD",
            "s": "BACKWARD",
            "a": "TURN_LEFT",
            "d": "TURN_RIGHT",
            "q": "CURVE_LEFT",
            "e": "CURVE_RIGHT",
        }

        for key, mode in key_to_mode.items():
            if self._key_triggered(keys, key):
                self.mode = mode
                self.start_time = time.time()
                print(f"[MODE] {self.mode}")

        if keys.get(p.B3G_SPACE, 0) & p.KEY_WAS_TRIGGERED:
            self.mode = "STOP"
            print("[MODE] STOP")

        if self._key_triggered(keys, "r"):
            self.robot.resetBody()
            self.origin_position = np.array(self.robot.getPos(), dtype=float)
            self.start_time = time.time()
            self.mode = "STOP"
            print("[RESET] 로봇 위치와 측정 기준을 초기화했습니다.")

        if self._key_triggered(keys, "p"):
            self.capture_screenshot()

    def read_parameters(self) -> dict[str, float]:
        """GUI 슬라이더의 현재 파라미터 값을 읽습니다."""
        return {
            "step_length": p.readUserDebugParameter(self.step_length_id),
            "step_height": p.readUserDebugParameter(self.step_height_id),
            "t1": p.readUserDebugParameter(self.stance_time_id),
            "t3": p.readUserDebugParameter(self.swing_time_id),
            "step_alpha": p.readUserDebugParameter(self.step_alpha_id),
            "body_height": p.readUserDebugParameter(self.body_height_id),
        }

    def make_command(self, parameters: dict[str, float]) -> dict[str, float]:
        """현재 모드와 슬라이더 값으로 TrottingGait 명령을 생성합니다."""
        length_sign, alpha_sign = MODE_COMMANDS[self.mode]

        return {
            "IDstepLength": length_sign * parameters["step_length"],
            "IDstepWidth": 0.0,
            "IDstepAlpha": alpha_sign * parameters["step_alpha"],
        }

    def apply_walking(self, parameters: dict[str, float]) -> None:
        """Trot 보행 궤적 또는 정지 자세를 로봇에 적용합니다."""
        self.gait.Sh = parameters["step_height"]
        self.gait.t1 = max(1, int(parameters["t1"]))
        self.gait.t3 = max(1, int(parameters["t3"]))

        self.robot.bodyPosition(
            (0.0, 40.0 + parameters["body_height"], 0.0)
        )

        if self.mode == "STOP":
            self.robot.feetPosition(self.stand_positions)
            return

        command = self.make_command(parameters)
        elapsed = time.time() - self.start_time
        foot_positions = self.gait.positions(elapsed, command)
        self.robot.feetPosition(foot_positions)

    def update_status(self, parameters: dict[str, float]) -> None:
        """이동 거리와 자세를 PyBullet 화면에 표시합니다."""
        body_position = np.array(self.robot.getPos(), dtype=float)
        body_orientation, _, _ = self.robot.getIMU()
        _, _, yaw = p.getEulerFromQuaternion(body_orientation)

        distance = float(
            np.linalg.norm(body_position[:2] - self.origin_position[:2])
        )
        yaw_degree = math.degrees(yaw)

        status = (
            f"Mode: {self.mode}\n"
            f"Distance: {distance:.3f} m | Yaw: {yaw_degree:.1f} deg\n"
            f"Sl={parameters['step_length']:.0f}  "
            f"Sh={parameters['step_height']:.0f}  "
            f"t1={parameters['t1']:.0f}  "
            f"t3={parameters['t3']:.0f}  "
            f"Sa={parameters['step_alpha']:.0f}"
        )

        text_position = [
            float(body_position[0]),
            float(body_position[1]),
            0.70,
        ]
        self.status_text_id = p.addUserDebugText(
            status,
            text_position,
            textColorRGB=[1.0, 1.0, 1.0],
            textSize=1.1,
            replaceItemUniqueId=self.status_text_id,
        )

    def capture_screenshot(self) -> None:
        """현재 PyBullet 디버그 카메라 화면을 PNG로 저장합니다."""
        try:
            import matplotlib.pyplot as plt

            camera = p.getDebugVisualizerCamera()
            width, height = int(camera[0]), int(camera[1])
            view_matrix, projection_matrix = camera[2], camera[3]

            if width <= 0 or height <= 0:
                raise RuntimeError("PyBullet 창 크기를 읽을 수 없습니다.")

            rgba = p.getCameraImage(
                width=width,
                height=height,
                viewMatrix=view_matrix,
                projectionMatrix=projection_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL,
            )[2]

            image = np.asarray(rgba, dtype=np.uint8).reshape(
                height, width, 4
            )
            filename = SCREENSHOT_FILES[self.mode]
            output_path = SCREENSHOT_DIR / filename
            plt.imsave(output_path, image)

            print(f"[SCREENSHOT] {output_path}")
        except Exception as error:
            print(f"[SCREENSHOT ERROR] 화면 저장 실패: {error}")

    def run(self) -> None:
        """시뮬레이션 메인 루프입니다."""
        try:
            while p.isConnected():
                self.process_keyboard()
                parameters = self.read_parameters()
                self.apply_walking(parameters)
                self.robot.step()
                self.update_status(parameters)
                time.sleep(1.0 / 240.0)
        except KeyboardInterrupt:
            print("\n[END] 시뮬레이션을 종료합니다.")
        finally:
            if p.isConnected():
                p.disconnect()


if __name__ == "__main__":
    Week07WalkingSimulation().run()