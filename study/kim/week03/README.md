# Week 03: PyBullet API 실습

## 실습 목표

- PyBullet의 stepSimulation() 동작 이해
- setJointMotorControl2() API 사용
- Position Control, Velocity Control, Torque Control 실험
- positionGain, force, torque 값 변경 실험

## 실습 내용

KUKA iiwa 로봇팔을 PyBullet에서 불러와 240Hz 기준으로 시뮬레이션을 실행하였다.

`setJointMotorControl2()` API를 사용하여 다음 3가지 관절 제어 모드를 실험하였다.

1. Position Control
2. Velocity Control
3. Torque Control

## 실험 결과

- Position Control에서는 목표 각도와 현재 각도의 차이를 확인하였다.
- Velocity Control에서는 목표 속도 값을 변경하며 관절 속도 변화를 확인하였다.
- Torque Control에서는 직접 토크 값을 입력하여 관절 움직임 변화를 확인하였다.
- positionGain, force, torque 값을 조정하며 로봇팔의 응답 차이를 관찰하였다.

## 실행 방법

```bash
py -3.11 week03_pybullet_api.py