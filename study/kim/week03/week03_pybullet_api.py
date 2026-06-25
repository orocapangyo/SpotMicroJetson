import pybullet as p
import pybullet_data
import time
import math

# PyBullet 실행
p.connect(p.GUI)

# PyBullet 기본 예제 파일 경로 설정
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 중력 설정
p.setGravity(0, 0, -9.81)

# 시뮬레이션 속도: 240Hz
p.setTimeStep(1 / 240)

# 바닥 생성
p.loadURDF("plane.urdf")

# 로봇팔 생성
robot = p.loadURDF(
    "kuka_iiwa/model.urdf",
    basePosition=[0, 0, 0],
    useFixedBase=True
)

# 관절 개수 확인
num_joints = p.getNumJoints(robot)
print("관절 개수:", num_joints)

# 조작할 관절 번호
joint_id = 2

# 카메라 위치 설정
p.resetDebugVisualizerCamera(
    cameraDistance=2,
    cameraYaw=50,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0.5]
)

counter = 0
mode = 0

while True:
    counter += 1

    # 5초마다 제어 모드 변경
    if counter % (240 * 5) == 0:
        mode = (mode + 1) % 4

    joint_state = p.getJointState(robot, joint_id)
    current_position = joint_state[0]
    current_velocity = joint_state[1]

    # 1번: Position Control
    if mode == 0:
        target_position = math.sin(counter * 0.01) * 1.0

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target_position,
            force=100,
            positionGain=0.5,
            velocityGain=1.0
        )

        if counter % 240 == 0:
            print("Position Control | 목표 각도:", round(target_position, 2),
                  "현재 각도:", round(current_position, 2))

    # 2번: Velocity Control
    elif mode == 1:
        target_velocity = 3.0

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=100
        )

        if counter % 240 == 0:
            print("Velocity Control | 목표 속도:", target_velocity,
                  "현재 속도:", round(current_velocity, 2))

    # 3번: Torque Control
    elif mode == 2:
        # 기존 모터 끄기
        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL,
            force=0
        )

        torque = math.sin(counter * 0.02) * 50

        p.setJointMotorControl2(
            bodyUniqueId=robot,
            jointIndex=joint_id,
            controlMode=p.TORQUE_CONTROL,
            force=torque
        )

        if counter % 240 == 0:
            print("Torque Control | 입력 토크:", round(torque, 2),
                  "현재 각도:", round(current_position, 2))

    # 시뮬레이션 한 단계 진행
    p.stepSimulation()

    # 실제 시간처럼 천천히 보기
    time.sleep(1 / 240)