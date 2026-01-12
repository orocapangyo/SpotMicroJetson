import pybullet as p
import pybullet_data
import time
import numpy as np
from matlab_circle import TrajectoryGenerator
from matlab_kinematics import LegIK
import math
class PyBulletLegController:
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.leg_ik = LegIK()
        
        # 관절 인덱스 (URDF 구조에 따라 조정 필요)
        self.joint_indices = {
            'front_left': [2, 3, 5],     # shoulder(2), leg(3), foot(5)
            'front_right': [7, 8, 10],   # shoulder(7), leg(8), foot(10)
            'rear_left': [12, 13, 15],   # shoulder(12), leg(13), foot(15)
            'rear_right': [17, 18, 20]   # shoulder(17), leg(18), foot(20)
        }

        # PD 게인
        self.kp = 0.015
        self.kd = 0.5
        self.max_force = 15.0

    def set_leg_position(self, leg_name, target_pos):
        """
        한 다리의 발끝 위치를 설정

        Parameters:
        -----------
        leg_name : str
            다리 이름 ('front_left', 'front_right', 'rear_left', 'rear_right')
        target_pos : tuple
            목표 발끝 위치 (x, y, z) - 몸통 기준 로컬 좌표
        """
        # IK 계산
        theta1, theta2, theta3 = self.leg_ik.inverse_kinematics(*target_pos)
        angles = [theta1, theta2, theta3]
        
        # 관절 제어
        joint_ids = self.joint_indices[leg_name]
        for joint_id, angle in zip(joint_ids, angles):
            p.setJointMotorControl2(
                self.robot_id,
                joint_id,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force,
                positionGain=self.kp,
                velocityGain=self.kd
            )
  
def circular_trajectory_pybullet():
    """
    PyBullet에서 한 발 원형 궤적 구현
    """
    # PyBullet 초기화
    p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    orn = p.getQuaternionFromEuler([math.pi/30*0, 0*math.pi/50, 0])
   
    # Simulation Configuration
    useFixedBase = False
    init_oritentation=p.getQuaternionFromEuler([0, 0, 90.0])    
    init_position=[0, 0, 0.3]
    useMaximalCoordinates = False
    flag = p.URDF_USE_SELF_COLLISION
    #바닥 설정
    planeUid = p.loadURDF("plane_transparent.urdf", [0, 0, 0], orn)
    p.changeDynamics(planeUid, -1, lateralFriction=1)
    texUid = p.loadTexture("concrete.png")
    p.changeVisualShape(planeUid, -1, textureUniqueId=texUid)
    p.changeDynamics(planeUid, -1, lateralFriction=1)
    #로봇 설정
    robot = p.loadURDF("../urdf/spotmicroai_gen.urdf.xml", init_position,
                            init_oritentation,
                            useFixedBase= useFixedBase,
                            useMaximalCoordinates=useMaximalCoordinates,
                            flags=flag) #p.URDF_USE_IMPLICIT_CYLINDER)
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.changeDynamics(robot, -1, lateralFriction=0.8)
    # 컨트롤러 생성
    controller = PyBulletLegController(robot)

    # 궤적 생성기
    traj_gen = TrajectoryGenerator(
        center=(100, -100, -30),
        radius= 30,
        height_offset = 20
    )

    # 시뮬레이션 루프
    t = 0
    dt = 1/240
    try:
        while True:
            # 원형 궤적 계산
            x, y, z = traj_gen.circular_trajectory(t, period=3.0)

            # 앞왼쪽 다리만 움직임
            controller.set_leg_position('front_left', (x, y, z))

            # 나머지 다리는 고정
            for leg_name in ['front_right', 'rear_left', 'rear_right']:
                controller.set_leg_position(leg_name, (100, -100 if 'right' in leg_name else 100, -100))

            
            p.stepSimulation()
            time.sleep(dt)
            t += dt
            
    except KeyboardInterrupt:
        print("시뮬레이션 종료")
    finally:
        p.disconnect()

# 실행
circular_trajectory_pybullet()

"""
    Lp=np.array([[100,-100,100,1], \
                [100,-100,-100,1], \
                [-100,-100,100,1], \
                [-100,-100,-100,1]])

"""