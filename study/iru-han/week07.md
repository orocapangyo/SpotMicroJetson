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

control_pitch 가 0.5 <br>
<img width="264" height="262" alt="image" src="https://github.com/user-attachments/assets/bd60b2f4-6545-4428-ae73-06370ef301e7" /> <br>
<img width="151" height="157" alt="image" src="https://github.com/user-attachments/assets/b3ebc6cb-a850-4d49-bed6-536abba60078" /> <br>
<br><br>
control_pitch 가 -0.5 <br>
<img width="162" height="180" alt="image" src="https://github.com/user-attachments/assets/cdddbef1-7303-4886-abe2-c65b02cf757c" /><br>
<img width="285" height="220" alt="image" src="https://github.com/user-attachments/assets/444aa85c-29ca-4255-9cf4-c4e32904432e" /><br>
<img width="181" height="144" alt="image" src="https://github.com/user-attachments/assets/5bdf4057-3a37-4cab-9ab8-4727a025805a" /><br>
<br><br>


control_roll 이 0.5  
<img width="147" height="186" alt="image" src="https://github.com/user-attachments/assets/38f14577-d411-4bc8-aed8-9223544f3915" />  <br>
<img width="149" height="143" alt="image" src="https://github.com/user-attachments/assets/1ecad31e-8de8-43d8-b632-83d2539702ec" />  <br>
<br><br>
control_roll 이 -0.5  
<img width="201" height="224" alt="image" src="https://github.com/user-attachments/assets/cc817c7b-b457-420e-a3cf-f586b57c951b" />  <br>
<img width="106" height="108" alt="image" src="https://github.com/user-attachments/assets/acb6e565-3cce-425a-bc48-78fdcf425ad4" />  <br>
<br><br>

com_offset_x 가 50  
<img width="265" height="191" alt="image" src="https://github.com/user-attachments/assets/4f994c2d-d9d6-425b-8a7c-dad9dbc76746" />  <br>
<img width="229" height="183" alt="image" src="https://github.com/user-attachments/assets/49770b48-d56d-4eac-b7cd-59a9d8cf4548" />  <br>
<img width="134" height="148" alt="image" src="https://github.com/user-attachments/assets/502155e9-f990-4ac1-bcf5-0849b88a2627" />  <br>
<br><br>

com_offset_z 가 20 (30 이상으로 하면 옆으로 넘어져버림)  
<img width="175" height="201" alt="image" src="https://github.com/user-attachments/assets/401709db-ed36-4ee2-8dcf-91a1a0c81159" />  <br>
<img width="150" height="174" alt="image" src="https://github.com/user-attachments/assets/c045f0fa-1f5c-44cc-be2f-f47b2668031f" />  <br>
<br><br>
