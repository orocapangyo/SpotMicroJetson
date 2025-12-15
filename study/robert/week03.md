# Week 03: PyBullet API 마스터하기

PyBullet의 stepSimulation, setJointMotorControl2 API를 마스터합니다. Position/Velocity/Torque 제어 모드를 실험하고 PD 게인 튜닝을 배웁니다.

## 학습 목표

1. PyBullet 시뮬레이션의 기본 동작 원리 이해
2. 3가지 관절 제어 모드 마스터
3. PD 게인 튜닝 방법 습득
4. 중력 보상 토크 계산 능력 배양

## 1. 시뮬레이션 기초

### 1.1 stepSimulation()
- 시뮬레이션을 한 타임스텝 진행
- 물리 엔진이 힘, 토크, 충돌 등을 계산
- 일반적으로 240Hz (1/240초) 간격으로 호출

```python
import pybullet as p
import time

# 시뮬레이션 초기화
p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

# 메인 루프
while True:
    p.stepSimulation()
    time.sleep(1./240.)
```

### 1.2 시뮬레이션 파라미터 설정
- `p.setTimeStep()` - 타임스텝 크기 설정
- `p.setGravity()` - 중력 설정
- `p.setPhysicsEngineParameter()` - 물리 엔진 세부 설정

## 2. 핵심 API

### 2.1 p.setJointMotorControl2()

관절을 제어하는 핵심 함수로, 3가지 제어 모드를 지원합니다.

#### (1) POSITION_CONTROL (위치 제어)
- 목표 각도로 관절을 이동
- 내부적으로 PD 제어기 사용
- 가장 많이 사용되는 모드

```python
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=jointIndex,
    controlMode=p.POSITION_CONTROL,
    targetPosition=target_angle,  # 목표 각도 (radians)
    force=maxForce,                # 최대 힘
    positionGain=Kp,               # P 게인
    velocityGain=Kd                # D 게인
)
```

#### (2) VELOCITY_CONTROL (속도 제어)
- 목표 속도로 관절을 회전
- 바퀴 구동 등에 적합

```python
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=jointIndex,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocity=target_velocity,  # 목표 속도 (rad/s)
    force=maxForce
)
```

#### (3) TORQUE_CONTROL (토크 제어)
- 직접 토크를 인가
- 가장 낮은 수준의 제어
- 중력 보상, 동역학 제어에 필수

```python
p.setJointMotorControl2(
    bodyUniqueId=robotId,
    jointIndex=jointIndex,
    controlMode=p.TORQUE_CONTROL,
    force=torque  # 인가할 토크
)
```

### 2.2 p.getJointState()

관절의 현재 상태를 읽어옵니다.

```python
joint_state = p.getJointState(robotId, jointIndex)
# 반환값: (position, velocity, reaction_forces, applied_torque)

position = joint_state[0]      # 현재 각도 (radians)
velocity = joint_state[1]      # 현재 각속도 (rad/s)
reaction = joint_state[2]      # 반력
applied_torque = joint_state[3]  # 적용된 토크
```

### 2.3 p.getLinkState()

링크(발끝 등)의 위치와 자세를 계산합니다.

```python
link_state = p.getLinkState(robotId, linkIndex)
# 반환값: (linkWorldPosition, linkWorldOrientation, ...)

position = link_state[0]  # 월드 좌표계에서의 위치 (x, y, z)
orientation = link_state[1]  # 월드 좌표계에서의 자세 (quaternion)
```

## 3. PD 게인 튜닝

### 3.1 PD 제어기의 원리

PD (Proportional-Derivative) 제어기는 위치 오차와 속도 오차를 이용하여 제어합니다.

```
τ = Kp × (θ_target - θ_current) + Kd × (ω_target - ω_current)
```

- **Kp (Position Gain)**: 위치 오차에 대한 반응
  - 크면: 빠르게 목표에 도달하지만 진동 발생
  - 작으면: 느리게 도달하지만 안정적

- **Kd (Velocity Gain)**: 속도 오차에 대한 반응 (댐핑)
  - 크면: 진동을 빠르게 감소
  - 작으면: 진동이 오래 지속

### 3.2 튜닝 방법

1. **Kp부터 시작**: Kd=0으로 설정하고 Kp를 점진적으로 증가
2. **진동 관찰**: 목표 위치 주변에서 진동이 시작하는 지점 확인
3. **Kd 추가**: 진동을 감소시키기 위해 Kd를 증가
4. **미세 조정**: 성능과 안정성의 균형을 찾을 때까지 반복

```python
# 예시: 게인 튜닝 실험
Kp_values = [0.1, 0.5, 1.0, 2.0, 5.0]
Kd_values = [0.01, 0.05, 0.1, 0.2, 0.5]

for Kp in Kp_values:
    for Kd in Kd_values:
        p.setJointMotorControl2(
            robotId, jointIndex,
            p.POSITION_CONTROL,
            targetPosition=target,
            positionGain=Kp,
            velocityGain=Kd
        )
```

## 4. 실습: Spot Mini 제어

### 4.1 Home Position으로 제어

Spot Mini를 서있는 자세로 만들기

```python
import pybullet as p
import pybullet_data
import time

# 초기화
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("spot_micro.urdf", [0, 0, 0.3])

# Home position 각도 정의 (예시)
home_positions = {
    'front_left_leg': [0.0, -0.8, 1.6],   # shoulder, elbow, wrist
    'front_right_leg': [0.0, -0.8, 1.6],
    'rear_left_leg': [0.0, -0.8, 1.6],
    'rear_right_leg': [0.0, -0.8, 1.6]
}

# 관절 인덱스 얻기
num_joints = p.getNumJoints(robot)
joint_indices = {}
for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_indices[joint_name] = i

# PD 게인 설정
Kp = 1.0
Kd = 0.1

# 제어 루프
for _ in range(10000):
    # 모든 관절을 home position으로 제어
    for leg, angles in home_positions.items():
        for i, angle in enumerate(angles):
            joint_name = f"{leg}_joint_{i}"
            if joint_name in joint_indices:
                p.setJointMotorControl2(
                    robot,
                    joint_indices[joint_name],
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=50,
                    positionGain=Kp,
                    velocityGain=Kd
                )

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
```

### 4.2 중력 보상 토크 계산

중력에 의한 관절 토크를 계산하고 보상합니다.

```python
# 1. 역동역학을 이용한 방법
def calculate_gravity_compensation(robot, joint_positions):
    """
    현재 자세에서 중력 보상 토크 계산
    """
    num_joints = p.getNumJoints(robot)

    # 영속도, 영가속도 상태에서의 토크 계산
    zero_velocities = [0.0] * num_joints
    zero_accelerations = [0.0] * num_joints

    gravity_compensation = p.calculateInverseDynamics(
        robot,
        joint_positions,
        zero_velocities,
        zero_accelerations
    )

    return gravity_compensation

# 2. 중력 보상 토크 적용
while True:
    # 현재 관절 각도 읽기
    joint_positions = []
    for i in range(p.getNumJoints(robot)):
        joint_state = p.getJointState(robot, i)
        joint_positions.append(joint_state[0])

    # 중력 보상 토크 계산
    gravity_torques = calculate_gravity_compensation(robot, joint_positions)

    # 토크 제어 모드로 적용
    for i in range(p.getNumJoints(robot)):
        p.setJointMotorControl2(
            robot, i,
            p.TORQUE_CONTROL,
            force=gravity_torques[i]
        )

    p.stepSimulation()
    time.sleep(1./240.)
```

### 4.3 발끝 위치 확인

```python
# 발끝 링크 인덱스 (URDF에 따라 다름)
foot_links = {
    'front_left': 3,
    'front_right': 7,
    'rear_left': 11,
    'rear_right': 15
}

# 발끝 위치 출력
for foot_name, link_index in foot_links.items():
    link_state = p.getLinkState(robot, link_index)
    position = link_state[0]
    print(f"{foot_name} foot position: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}")
```

## 5. 과제

1. **PD 게인 실험**: 다양한 Kp, Kd 값으로 관절 제어 응답 관찰
2. **제어 모드 비교**: POSITION, VELOCITY, TORQUE 모드의 차이 체험
3. **중력 보상**: 중력 보상이 있을 때와 없을 때의 차이 확인
4. **발끝 위치 제어**: 역기구학을 사용하여 발끝을 원하는 위치로 이동

## 6. 참고 자료

- [PyBullet Quickstart Guide](https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA)
- [PyBullet API Documentation](https://pybullet.org/wordpress/)
- PD Control Theory
- Inverse Dynamics for Gravity Compensation
