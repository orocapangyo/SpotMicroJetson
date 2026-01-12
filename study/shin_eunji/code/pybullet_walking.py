import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import sys
sys.path.append("..")

from Kinematics.kinematicMotion import TrottingGait
from Kinematics.kinematics import Kinematic

class PyBulletInPlaceWalking:
    """
    PyBullet에서 제자리 걷기 시뮬레이션
    """
    def __init__(self):
        # PyBullet 초기화
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        orn = p.getQuaternionFromEuler([math.pi/30*0, 0*math.pi/50, 0])
   
        # 지면 및 로봇 로드
        useFixedBase = False
        init_oritentation=p.getQuaternionFromEuler([0, 0, 90.0])    
        init_position=[0, 0, 0.3]
        useMaximalCoordinates = False
        flag = p.URDF_USE_SELF_COLLISION
        #바닥 설정
        self.plane = p.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)        
        p.changeDynamics(self.plane, -1, lateralFriction=1)
        texUid = p.loadTexture("concrete.png")
        p.changeVisualShape(self.plane, -1, textureUniqueId=texUid)
        p.changeDynamics(self.plane, -1, lateralFriction=1)
        #로봇 설정
        self.robot = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", init_position,
                                init_oritentation,
                                useFixedBase= useFixedBase,
                                useMaximalCoordinates=useMaximalCoordinates,
                                flags=flag) #p.URDF_USE_IMPLICIT_CYLINDER)

        # 보행 패턴 및 기구학
        self.trotting = TrottingGait()
        self.kinematics = Kinematic()

        self.joint_indices = {
            'FL': [2, 3, 5],     # shoulder(2), leg(3), foot(5)
            'FR': [7, 8, 10],   # shoulder(7), leg(8), foot(10)
            'RL': [12, 13, 15],   # shoulder(12), leg(13), foot(15)
            'RR': [17, 18, 20]   # shoulder(17), leg(18), foot(20)
        }

        # PD 제어 파라미터
        self.kp = 0.02
        self.kd = 0.3
        self.max_force = 15

        # 시작 시간
        self.start_time = time.time()

        # GUI 슬라이더 추가
        self.height_slider = p.addUserDebugParameter("Height", -50, 50, 20)
        self.step_height_slider = p.addUserDebugParameter("Step Height", 0, 100, 40)

    def get_foot_positions(self, t):
        """
        시간 t에서 발끝 위치 반환 (제자리 걷기)
        """
        kb_offset = {
            'IDstepLength': 0.0,
            'IDstepWidth': 0.0,
            'IDstepAlpha': 0.0
        }
        return self.trotting.positions(t, kb_offset)

    def foot_to_angles(self, foot_pos):
        """
        발끝 위치를 관절 각도로 변환 (IK)
        """
        x, y, z, _ = foot_pos
        angles = self.kinematics.legIK([x, y, z, 1])
        return angles

    def set_leg_angles(self, leg_name, angles):
        """
        한 다리의 관절 각도 설정
        """
        joint_ids = self.joint_indices[leg_name]
        for joint_id, angle in zip(joint_ids, angles):
            p.setJointMotorControl2(
                self.robot,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force,
                positionGain=self.kp,
                velocityGain=self.kd
            )

    def step(self):
        """
        한 시뮬레이션 스텝 실행
        """
        t = time.time() - self.start_time

        # GUI에서 파라미터 읽기
        height = p.readUserDebugParameter(self.height_slider)
        step_height = p.readUserDebugParameter(self.step_height_slider)
        self.trotting.Sh = step_height

        # 발끝 위치 계산
        foot_positions = self.get_foot_positions(t)
        leg_names = ['FL', 'FR', 'RL', 'RR']

        # 각 다리에 IK 적용
        for leg_name, foot_pos in zip(leg_names, foot_positions):
            try:
                # 높이 보정
                print(leg_name,foot_pos)
                foot_pos_adjusted = foot_pos.copy()
                foot_pos_adjusted[1] += height

                angles = self.foot_to_angles(foot_pos_adjusted)
                self.set_leg_angles(leg_name, angles)
            except Exception as e:
                print(f"IK error for {leg_name}: {e}")

        p.stepSimulation()

    def run(self, duration=60):
        """
        시뮬레이션 실행
        """
        print("="*50)
        print("제자리 걷기 시뮬레이션 시작")
        print("Ctrl+C로 종료")
        print("="*50)

        try:
            while True:
                self.step()
                time.sleep(1./240.)
        except KeyboardInterrupt:
            print("\n시뮬레이션 종료")
        finally:
            p.disconnect()

# 실행
if __name__ == "__main__":
    sim = PyBulletInPlaceWalking()
    sim.run()