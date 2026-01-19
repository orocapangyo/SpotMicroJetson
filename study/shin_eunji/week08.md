8주차 - mujoco 환경 준비 

과제
- URDF를 MJCF로 변환
- MuJoCo 시뮬레이션 구동
- PyBullet vs MuJoCo 비교

1. pip install mujoco  설치
2. mujoco 환경에서는 로봇과 환경 모델이 통합해야 되므로 통합 환경을 제작

https://blog.naver.com/silverbjin/223935860235

### **URDF를 MJCF로 먼저 변환**

urdf2mjcf와 같은 도구를 사용하면 URDF 파일을 MuJoCo XML(MJCF)로 손쉽게 변환할 수 있다

문제 :  STL이 빠져서 파일이 생성된다

urdf 파일 내부에 생성하기 

```xml
 <mujoco>
    <compiler discardvisual="false" strippath="false" fuseparent="false"/>
 </mujoco>
```

```python
1. urdf2mjcf
urdf2mjcf spotmicroai_gen.urdf.xml
2.파일을 viewer로 연다음 저장
python3 -m mujoco.viewer --mjcf=spotmicroai_gen.urdf.xml
```

배경도 xml 에 같이 넣기

```xml
    <texture name="texplane" type="2d" builtin="checker" rgb1=".0 .0 .0" rgb2=".8 .8 .8" width="100" height="100" />
    <material name="matplane" reflectance="0." texture="texplane" texrepeat="1 1" texuniform="true" />
    <material name="visualgeom" rgba="0.5 0.9 0.2 1" />
```

**파일 저장:** 상단 메뉴에서 **File → Save XML**을 선택하여 저장한다.

spotmicro_gen_mujoco.xml 생성 

basic  파일 만들어서 만든 xml 파일을 구현해보기

```python
import mujoco
import mujoco.viewer
import numpy as np

class SpotSimulation:
    def __init__(self):
        self.model = None
        self.data = None
        self.init_position = [0, 0, 0.5]  # 예시 위치
        self.init_orientation = [1, 0, 0, 0] # MuJoCo는 Quaternion이 [w, x, y, z]입니다.

    def loadModels(self):
        # 1. 모델 로드 (world.xml에 로봇과 바닥이 모두 포함되어 있다고 가정)
        # 만약 URDF를 직접 로드하려면 path를 URDF로 지정하세요.
        self.model = mujoco.MjModel.from_xml_path("/home/ehdtod001009/micro_spot/SpotMicroJetson/urdf/MUJOCO_MODEL/mujoco_env.xml")
        self.data = mujoco.MjData(self.model)

        # 2. 초기 위치 및 자세 설정 (PyBullet의 loadURDF 인자 역할)
        # qpos의 처음 7개 값은 보통 Freejoint(3축 위치 + 4축 쿼터니언)입니다.
        self.data.qpos[0:3] = self.init_position
        self.data.qpos[3:7] = self.init_orientation

        # 3. 중력 설정 (기본적으로 MJCF에 설정되지만 코드에서 수정 가능)
        self.model.opt.gravity = [0, 0, -9.81]

        # 4. 마찰력 설정 (PyBullet의 changeDynamics 역할)
        # geom_friction 인덱스를 찾아 직접 수정하거나 XML에서 설정하는 것이 권장됩니다.
        # 예: 모든 geom의 마찰력을 0.8로 설정
        self.model.geom_friction[:] = 0.8

        mujoco.mj_forward(self.model, self.data)
        return self.model, self.data

    def run(self):
        self.loadModels()
        # 뷰어 실행
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(self.model, self.data)
                viewer.sync()

# 실행
sim = SpotSimulation()
sim.run()
```

문제2: toe.stl이 가져오는데 오류가 생김 

문제3: 관성 설정이 이상하게 되어 있어서 애가 서있지를 못함 

### MuJoCo 시뮬레이션 구동

![image.png](attachment:48d58b7b-fbac-46e5-a365-64fe9969174f:image.png)

### PyBullet vs MuJoCo 비교

pybullet은 **Bullet Physics Engine**을 사용, 물리적인 정확성보다는 **속도**와 **간단함**을 중시

mujoco는 특히 **정확한 물리학적 모델링**을 위한 엔진으로 설계

그러기에 Mujoco는 PyBullet보다 속도가 느림 

mujoco 옵션 확인

조인트 각도 실시간으로 확인과 제어가능 

![image.png](attachment:5fe47328-aef7-4c01-a571-e34a80fcbfb1:image.png)

Control에 있는 조인트 움직여보기 

![image.png](attachment:1a2b77e1-16c1-4701-8151-e765f3ed933d:image.png)

![image.png](attachment:0a9c5da8-63a4-41b3-bacc-215b1b00cb39:image.png)

Option에서 Sensor data 확인까지 가능

![image.png](attachment:b279e07f-7d08-4ce2-b443-309d4358c025:image.png)

![image.png](attachment:904d1b61-7cc8-4c4a-a0d5-0583e194292b:image.png)
