import sys
sys.path.append("..")

import numpy as np
import time
import math
from Kinematics.kinematicMotion import TrottingGait

class InPlaceWalking:
    """
    제자리 걷기 시뮬레이션 클래스
    """
    def __init__(self):
        self.trotting = TrottingGait()

        # 제자리 걷기 설정 (보폭 = 0)
        self.kb_offset = {
            'IDstepLength': 0.0,  # 전후 보폭 없음
            'IDstepWidth': 0.0,   # 좌우 이동 없음
            'IDstepAlpha': 0.0    # 회전 없음
        }

        # 시작 시간
        self.start_time = time.time()

    def get_foot_positions(self):
        """
        현재 시간에서 4개 다리의 발끝 위치 반환
        """
        current_time = time.time() - self.start_time
        positions = self.trotting.positions(current_time, self.kb_offset)
        return positions

    def print_positions(self):
        """
        발끝 위치를 콘솔에 출력
        """
        positions = self.get_foot_positions()
        leg_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']

        print("\n" + "="*60)
        print(f"Time: {time.time() - self.start_time:.2f}s")
        print("-"*60)
        for i, (name, pos) in enumerate(zip(leg_names, positions)):
            print(f"{name:12} | X: {pos[0]:7.2f} | Y: {pos[1]:7.2f} | Z: {pos[2]:7.2f}")
        print("="*60)


# 실행 예시
if __name__ == "__main__":
    walker = InPlaceWalking()

    try:
        for _ in range(50):
            walker.print_positions()
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n시뮬레이션 종료")