pybullet_automatic_gait.py 를 다음과 같이 수정했을 때 로봇이 회전을 하며 이동했다
```
def main():
    # 1. 로봇 생성 (중요: useFixedBase=False로 해야 움직임)
    robot = spotmicroai.Robot(useFixedBase=False, useStairs=True, resetFunc=reset)

    # 파라미터 설정
    IDheight = p.addUserDebugParameter("Body Height", -40, 90, 20)

    # 기본 발 위치
    spurWidth = robot.W/2 + 20
    iXf = 120
    Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
                   [-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

    trotting = TrottingGait()

    print("Simulation Start...")

    while True:
        # 로봇 상태 읽기
        bodyPos = robot.getPos()
        bodyOrn, _, _ = robot.getIMU()
        xr, yr, _ = p.getEulerFromQuaternion(bodyOrn)

        # 멀리 가면 리셋
        if math.sqrt(bodyPos[0]**2 + bodyPos[1]**2) > 50:
            robot.resetBody()
            reset()

        d = time.time() - rtime
        height = p.readUserDebugParameter(IDheight)
        ir = xr / (math.pi/180)

        # ====================================================
        # [과제 해결용 파라미터 조절 구간]
        # 여기 숫자를 바꾸면 로봇의 움직임이 변합니다.
        # ====================================================

        target_step_length = 30  # 전진 속도 (양수: 전진, 0: 정지)
        target_rotation = 15     # 회전 각도 (양수: 좌회전, 음수: 우회전)

        # ====================================================

        # 명령 생성
        cmd = {
            'IDstepLength': target_step_length,
            'IDstepWidth': 0,
            'IDstepAlpha': target_rotation
        }

        # 보행 알고리즘 실행 (무조건 걷게 설정)
        robot.feetPosition(trotting.positions(d, cmd))

        # 몸통 자세 제어 (수평 유지)
        robot.bodyRotation((0, 0, 0))

        # 몸통 높이 및 앞뒤 기울기 보정
        bodyX = 50 + yr * 10
        robot.bodyPosition((bodyX, 40 + height, -ir))

        # 시뮬레이션 스텝 진행
        robot.step()
```
