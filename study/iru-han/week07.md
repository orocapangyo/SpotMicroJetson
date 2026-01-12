pybullet_automatic_gait.py 를 다음과 같이 수정했을 때 로봇이 회전을 하며 이동했다
```
"""
Automatic Gait Simulation (Final Homework Version)
"""
import sys
sys.path.append("..")
import numpy as np
import time
import math
import pybullet as p
import spotmicroai
from kinematicMotion import TrottingGait

# 전역 타이머 리셋 함수
rtime = time.time()
def reset_timer():
    global rtime
    rtime = time.time()

def main():
    # [설정 1] 로봇 생성 (useFixedBase=False 필수)
    robot = spotmicroai.Robot(useFixedBase=False, useStairs=False, resetFunc=reset_timer)

    # [설정 2] 디버그 슬라이더 설정
    IDheight = p.addUserDebugParameter("Body Height", -40, 90, 20)

    # [설정 3] 초기 발 위치
    spurWidth = robot.W/2 + 20
    iXf = 120
    Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
                   [-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

    trotting = TrottingGait()

    print("Simulation Start... (Homework Mode)")

    while True:
        # 로봇 상태 읽기
        bodyPos = robot.getPos()
        bodyOrn, _, _ = robot.getIMU()
        xr, yr, _ = p.getEulerFromQuaternion(bodyOrn)

        # 리셋 로직 (너무 멀리 가면 원위치)
        if math.sqrt(bodyPos[0]**2 + bodyPos[1]**2) > 50:
            robot.resetBody()
            reset_timer()

        d = time.time() - rtime
        height = p.readUserDebugParameter(IDheight)
        ir = xr / (math.pi/180)

        # ====================================================
        # 아래 변수들의 숫자를 바꿔가며 캡처.
        # ====================================================

        # 1. 보행 제어 (전진 및 회전)
        target_step_length = 30   # 보폭 (0이면 제자리, 30이면 전진)
        target_rotation = 15      # 회전 (0이면 직진, 15면 좌회전)

        # 2. 자세 제어 (Body Pitch / Roll)
        # 0.0은 수평. 0.3이나 -0.3 등을 넣기.
        control_pitch = 0      # 앞뒤 기울기
        control_roll = 0       # 좌우 기울기

        # 3. CoM (무게중심) 추적 제어
        # 몸통(Body)을 다리 중심에서 강제로 밀어버리는 오프셋
        # 0이면 정상, 50 이상이면 앞으로 쏠려서 넘어질 수 있음
        com_offset_x = 0         # 앞뒤 무게중심 이동
        com_offset_z = 0         # 좌우 무게중심 이동

        # ====================================================

        # 보행 명령 생성
        cmd = {
            'IDstepLength': target_step_length,
            'IDstepWidth': 0,
            'IDstepAlpha': target_rotation
        }

        # 로봇 발 위치 제어 (Inverse Kinematics)
        robot.feetPosition(trotting.positions(d, cmd))

        # Body Rotation 제어 적용
        # 위에서 설정한 pitch, roll 값을 로봇에게 전달
        robot.bodyRotation((control_roll, control_pitch, 0))

        # Body Position (CoM) 제어 적용
        # CoM 위치 = 기본 위치 + 오프셋
        final_x = 50 + (yr * 10) + com_offset_x
        final_z = -ir + com_offset_z

        robot.bodyPosition((final_x, 40 + height, final_z))

        # 물리 시뮬레이션 1스텝 진행
        robot.step()

if __name__ == "__main__":
    main()
```
