# Week08 - MuJoCo 시뮬레이션 및 PyBullet 비교

## 1. 학습 목표

이번 Week08에서는 기존에 사용했던 PyBullet과 함께 MuJoCo 물리 시뮬레이션 환경을 구성하고, SpotMicro 모델을 MuJoCo에서 실행해 보았다.

주요 학습 목표는 다음과 같다.

- MuJoCo 설치 및 실행 환경 구성
- MuJoCo에서 SpotMicro 모델 로딩
- MuJoCo 물리 시뮬레이션 실행
- PyBullet과 MuJoCo 시뮬레이션 성능 비교
- 두 시뮬레이터의 특징 및 차이점 확인


---

## 2. MuJoCo 설치 확인

Python 환경에서 MuJoCo가 정상적으로 설치되었는지 확인하였다.

```bash
python -c "import mujoco; print(mujoco.__version__)"