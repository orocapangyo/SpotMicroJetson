# SpotMicro Week 07 — PyBullet 전진·회전 보행 및 파라미터 제어

> 작성일: 2026-08-06

---

## 1. 실습 개요

이번 실습에서는 PyBullet 환경에서 SpotMicro 사족보행 로봇의 Trot 보행을 구현하였다.

주요 구현 항목은 다음과 같다.

- 전진 및 후진 보행
- 제자리 좌회전 및 우회전
- 전진하면서 좌우로 이동하는 커브 주행
- 키보드를 이용한 실시간 주행 모드 변경
- 보폭, 발 높이, 보행 시간, 회전각 파라미터 조정
- 이동 거리와 Yaw 각도 확인
- 실행 화면 이미지 저장

실행 파일:

```text
study/bangsajang-cmyk/week07/week07_trot_control.py
```

---

## 2. 실행 환경

| 항목 | 내용 |
|---|---|
| 운영체제 | Windows |
| 개발 환경 | Visual Studio Code |
| Python | Python 3.14 |
| 물리 시뮬레이터 | PyBullet |
| GPU | NVIDIA GeForce GTX 1060 3GB |
| 실행 방식 | PyBullet GUI |
| 로봇 모델 | SpotMicro |

실행 명령:

```powershell
python study/bangsajang-cmyk/week07/week07_trot_control.py
```

---

## 3. 구현 과정

### 3.1 프로젝트 경로 설정

실행 위치와 관계없이 프로젝트의 `Kinematics`와 `Simulation` 모듈을 불러올 수 있도록 현재 파일 위치를 기준으로 프로젝트 루트를 설정하였다.

```python
SCRIPT_PATH: Final[Path] = Path(__file__).resolve()
PROJECT_ROOT: Final[Path] = SCRIPT_PATH.parents[3]
SCREENSHOT_DIR: Final[Path] = SCRIPT_PATH.parent / "images"

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
```

프로젝트 구조:

```text
SpotMicroJetson/
├─ Kinematics/
├─ Simulation/
└─ study/
   └─ bangsajang-cmyk/
      ├─ work07.md
      └─ week07/
         ├─ week07_trot_control.py
         └─ images/
```

### 3.2 PyBullet 경로 문제 해결

Windows 사용자 폴더에 한글 경로가 포함된 경우 PyBullet이 `plane.urdf` 또는 `plane_transparent.urdf` 경로를 정상적으로 읽지 못하는 문제가 발생하였다.

기존 URDF 평면 파일 대신 PyBullet의 기본 평면 충돌 형상을 직접 생성하는 방식으로 해결하였다.

```python
plane_collision_id = p.createCollisionShape(
    shapeType=p.GEOM_PLANE
)

plane_uid = p.createMultiBody(
    baseMass=0,
    baseCollisionShapeIndex=plane_collision_id,
    basePosition=[0, 0, 0],
)

p.changeDynamics(
    plane_uid,
    -1,
    lateralFriction=1.0,
)
```

또한 `Simulation/__init__.py`의 모듈 경로를 실제 폴더 구조에 맞게 수정하였다.

```python
from Kinematics.kinematics import Kinematic
```

---

## 4. 키보드 주행 제어

PyBullet 창의 3D 화면을 클릭한 후 영문 입력 상태에서 키를 사용한다.

| 키 | 기능 |
|---|---|
| `W` | 전진 |
| `S` | 후진 |
| `A` | 제자리 왼쪽 회전 |
| `D` | 제자리 오른쪽 회전 |
| `Q` | 전진하면서 왼쪽 커브 |
| `E` | 전진하면서 오른쪽 커브 |
| `Space` | 정지 |
| `R` | 로봇 위치와 측정값 초기화 |
| `P` | 현재 PyBullet 화면 이미지 저장 |

주행 모드별 명령값:

```python
MODE_COMMANDS = {
    "STOP": (0.0, 0.0),
    "FORWARD": (1.0, 0.0),
    "BACKWARD": (-1.0, 0.0),
    "TURN_LEFT": (0.0, -1.0),
    "TURN_RIGHT": (0.0, 1.0),
    "CURVE_LEFT": (1.0, -1.0),
    "CURVE_RIGHT": (1.0, 1.0),
}
```

첫 번째 값은 전진·후진 방향을 결정하고, 두 번째 값은 좌우 회전 방향을 결정한다.

---

## 5. 보행 파라미터

PyBullet 오른쪽 슬라이더에서 다음 값을 실시간으로 조절할 수 있도록 구현하였다.

| 파라미터 | 기본값 | 기능 |
|---|---:|---|
| `Step Length Sl` | 60 mm | 한 걸음의 전후 이동 거리 |
| `Step Height Sh` | 40 mm | 발을 들어 올리는 높이 |
| `Stance Time t1` | 1200 ms | 발이 지면을 지지하는 시간 |
| `Swing Time t3` | 200 ms | 발을 앞으로 이동시키는 시간 |
| `Step Alpha Sa` | 15 deg | 좌우 회전량 |
| `Body Height Offset` | 20 mm | 몸체 높이 보정값 |

### 5.1 파라미터 변화 예상

| 변경 항목 | 값 증가 시 예상되는 변화 |
|---|---|
| Step Length | 이동 속도 증가, 과도하면 자세 불안정 가능 |
| Step Height | 장애물 통과에 유리, 과도하면 몸체 진동 증가 |
| Stance Time | 값이 커지면 보행 주기가 느려짐 |
| Swing Time | 값이 커지면 발 이동이 느리고 부드러워짐 |
| Step Alpha | 회전 속도 증가, 과도하면 미끄러짐 가능 |
| Body Height Offset | 몸체 높이가 높아지거나 낮아짐 |

---

## 6. 실행 결과

### 6.1 PyBullet 초기 실행 화면

로봇 모델과 오른쪽 파라미터 슬라이더가 정상적으로 표시되었다.

<img width="1024" height="768" alt="parameter_test" src="https://github.com/user-attachments/assets/385a7a2e-41ac-4af1-b860-13ee00b0d1df" />

![PyBullet 초기 실행 및 파라미터 화면](week07/images/parameter_test.png)

### 6.2 전진 보행

`W` 키를 눌러 전진 모드로 변경하였다. 화면의 모드가 `FORWARD`로 변경되고 Trot 보행을 이용하여 전진하는 것을 확인하였다.

<img width="1024" height="768" alt="forward_walking" src="https://github.com/user-attachments/assets/68f42aa2-9226-4162-b5aa-584b151444de" />

![SpotMicro 전진 보행](week07/images/forward_walking.png)

### 6.3 좌우 회전

`A` 또는 `D` 키를 눌러 제자리 좌회전과 우회전을 수행하였다. `Step Alpha` 값에 따라 회전 속도와 회전량이 변화하였다.

<img width="1024" height="768" alt="turning_walk" src="https://github.com/user-attachments/assets/afcc6502-fb7c-4a16-87c1-1ff8f3c67e8b" />

![SpotMicro 제자리 회전](week07/images/turning_walk.png)

### 6.4 커브 주행

`Q` 또는 `E` 키를 눌러 전진과 회전을 동시에 적용하였다. 전진 명령과 회전 명령을 결합하여 좌우 커브 주행을 구현하였다.

<img width="1024" height="768" alt="curve_walking" src="https://github.com/user-attachments/assets/0efc2e1a-825f-4ac8-a580-020a752be4ee" />

![SpotMicro 커브 주행](week07/images/curve_walking.png)

---

## 7. 보행 시험 결과

실행 결과를 관찰하여 다음과 같이 정리하였다.

| 시험 항목 | 사용 키 | 확인 결과 |
|---|---|---|
| 정지 자세 | `Space` | 정상 |
| 전진 보행 | `W` | 정상 |
| 후진 보행 | `S` | 정상 |
| 제자리 좌회전 | `A` | 정상 |
| 제자리 우회전 | `D` | 정상 |
| 좌측 커브 | `Q` | 정상 |
| 우측 커브 | `E` | 정상 |
| 위치 초기화 | `R` | 정상 |
| 이미지 저장 | `P` | 정상 |

### 7.1 파라미터 시험 기록

실제 시험 후 측정값이나 관찰 내용을 아래 표에 추가한다.

| 시험 번호 | Sl | Sh | t1 | t3 | Sa | 결과 |
|---:|---:|---:|---:|---:|---:|---|
| 1 | 60 | 40 | 1200 | 200 | 15 | 기본값에서 정상 보행 |
| 2 | 80 | 40 | 1200 | 200 | 15 | 실행 후 결과 입력 |
| 3 | 60 | 60 | 1200 | 200 | 15 | 실행 후 결과 입력 |
| 4 | 60 | 40 | 900 | 200 | 15 | 실행 후 결과 입력 |
| 5 | 60 | 40 | 1200 | 200 | 20 | 실행 후 결과 입력 |

## 9. 파일 구성

최종 제출 파일 구조는 다음과 같다.

```text
study/bangsajang-cmyk/
├─ work07.md
└─ week07/
   ├─ week07_trot_control.py
   └─ images/
      ├─ parameter_test.png
      ├─ forward_walking.png
      ├─ turning_walk.png
      └─ curve_walking.png
```

---

## 10. 결론

PyBullet 환경에서 SpotMicro의 Trot 보행을 이용하여 전진, 후진, 제자리 좌우 회전, 좌우 커브 주행을 구현하였다.

키보드 입력을 통해 주행 모드를 실시간으로 변경할 수 있었으며, GUI 슬라이더를 이용하여 보폭, 발 높이, 보행 주기, 회전각, 몸체 높이를 조절할 수 있었다.

보폭과 회전각이 증가하면 이동 및 회전 반응이 커졌지만, 값이 지나치게 크면 발 미끄러짐이나 몸체 자세 불안정이 발생할 가능성이 있으므로 안정적인 범위에서 조절해야 한다.

---

## 11. 다음 단계

- [ ] 실제 시험값을 파라미터 시험표에 기록
- [ ] 전진 보행 사진 업로드
- [ ] 제자리 회전 사진 업로드
- [ ] 커브 주행 사진 업로드
- [ ] 파라미터 화면 사진 업로드
- [ ] 실행 영상 PR 설명에 첨부
- [ ] 코드 PR과 문서 PR 분리 제출
