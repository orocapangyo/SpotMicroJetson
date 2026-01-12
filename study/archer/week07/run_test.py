"""
7-3-bound.py: PyBullet Bounding Gait Simulation
Week 07 - Bound 보행 시뮬레이션 (강아지 달리기 모션)

이 시뮬레이션은 앞다리 2개, 뒷다리 2개가 동시에 움직이는 
Bound 보행 패턴을 구현합니다.

조작 방법:
- Step Length 슬라이더: 보폭 조절 (0~100mm)
- Step Height 슬라이더: 발 들어올리는 높이 (20~80mm)
- Height 슬라이더: 몸체 높이 조절
- W 키: 걷기 토글
- R 키: 로봇 위치 리셋
"""

import sys
import os

# 상위 디렉토리를 경로에 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(script_dir, "..", "..")
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.join(project_root, "Kinematics"))
sys.path.insert(0, os.path.join(project_root, "Simulation"))

import pybullet as p
import pybullet_data
import time
import math
import numpy as np

from Kinematics.kinematics import Kinematic


class BoundingGait:
    """
    Bounding Gait 패턴 생성기
    앞다리(FL+FR)와 뒷다리(RL+RR)가 각각 동시에 움직입니다.
    """
    
    def __init__(self):
        # 기본 발끝 위치 (로봇 몸체 기준)
        self.W = 75 + 5 + 40  # 좌우 폭
        
        # 기본 발 위치 [x, y, z, 1]
        # x: 앞뒤, y: 높이, z: 좌우
        self.default_stance = np.array([
            [120, -100, self.W/2, 1],    # Front Left (FL)
            [120, -100, -self.W/2, 1],   # Front Right (FR)
            [-50, -100, self.W/2, 1],    # Rear Left (RL)
            [-50, -100, -self.W/2, 1]    # Rear Right (RR)
        ])
        
        # 보행 파라미터
        self.Sh = 40.0  # Step Height (발 들어올리는 높이)
        self.period = 1.0  # 보행 주기 (초)
    
    def positions(self, t, kb_offset):
        """
        시간 t에서의 4개 다리 발끝 위치 계산
        
        Returns:
        --------
        foot_positions : numpy array (4x4)
            각 다리의 [x, y, z, 1] 위치
            x: 앞뒤(전진), y: 높이(수직), z: 좌우
        """
        step_length = kb_offset.get('IDstepLength', 0.0)
        step_alpha = kb_offset.get('IDstepAlpha', 0.0)
        
        # 보행 주기 계산
        phase = (t % self.period) / self.period  # 0~1 반복
        
        foot_positions = np.copy(self.default_stance)
        
        # 각 다리의 궤적 계산
        for leg_idx in range(4):
            # 앞다리(0,1) vs 뒷다리(2,3) 구분
            if leg_idx < 2:  # Front legs (FL, FR)
                leg_phase = phase
            else:  # Rear legs (RL, RR)
                leg_phase = (phase + 0.5) % 1.0  # 0.5 뒤쳐짐
            
            # 타원 궤적 계산
            x_offset, y_offset = self._compute_trajectory(
                leg_phase, 
                step_length,
                self.Sh
            )
            
            # 회전 적용
            if step_alpha != 0:
                # 앞다리는 바깥쪽, 뒷다리는 안쪽으로
                if leg_idx < 2:  # 앞다리
                    x_offset += step_alpha * 0.5
                else:  # 뒷다리
                    x_offset -= step_alpha * 0.5
            
            # ★ 핵심 수정: 좌표계 올바르게 적용 ★
            foot_positions[leg_idx][0] += x_offset  # X축: 앞뒤 이동
            foot_positions[leg_idx][1] += y_offset  # Y축: 높이 변화 (수정!)
            # Z축 (foot_positions[leg_idx][2])은 좌우이므로 건드리지 않음
        
        return foot_positions
    
    def _compute_trajectory(self, phase, step_length, step_height):
        """
        단일 다리의 타원 궤적 계산
        
        Returns:
        --------
        x_offset : float
            앞뒤 방향 오프셋 (전진 방향)
        y_offset : float
            높이 방향 오프셋 (수직 방향)
        """
        if phase < 0.5:
            # Swing Phase (공중): 발을 앞으로 이동
            swing_progress = phase / 0.5  # 0~1
            
            # X축: -step_length/2 → +step_length/2 (앞으로)
            x_offset = -step_length/2 + step_length * swing_progress
            
            # Y축: 포물선 (0 → step_height → 0) - 수직 위로!
            y_offset = step_height * math.sin(swing_progress * math.pi)
            
        else:
            # Stance Phase (지면): 발로 지면을 뒤로 밀기
            stance_progress = (phase - 0.5) / 0.5  # 0~1
            
            # X축: +step_length/2 → -step_length/2 (뒤로)
            x_offset = step_length/2 - step_length * stance_progress
            
            # Y축: 지면 접촉 (높이 0)
            y_offset = 0.0
        
        return x_offset, y_offset

class BoundingWalkingSimulation:
    """
    PyBullet Bounding 보행 시뮬레이션
    앞다리와 뒷다리가 각각 동시에 움직이는 패턴
    """
    
    def __init__(self):
        # PyBullet 초기화
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 시뮬레이션 파라미터
        p.setPhysicsEngineParameter(numSolverIterations=150)
        p.setRealTimeSimulation(True)
        
        # 지면 로드
        self.plane = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane, -1, lateralFriction=1.0)
        
        # SpotMicro 로봇 로드
        urdf_path = os.path.join(project_root, "urdf", "spotmicroai_gen.urdf.xml")
        self.init_position = [0, 0, 0.25]
        self.init_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        self.robot = p.loadURDF(
            urdf_path,
            self.init_position,
            self.init_orientation,
            useFixedBase=False,
            flags=p.URDF_USE_SELF_COLLISION
        )
        
        # 기구학 및 보행 패턴
        self.kinematics = Kinematic()
        self.bounding = BoundingGait()  # ← Bounding 패턴 사용!
        
        # 관절 매핑
        self.joint_name_to_id = self._get_joint_names()
        
        # 다리 방향 보정
        self.dirs = [[-1, 1, 1], [1, 1, 1], [-1, 1, 1], [1, 1, 1]]
        
        # PD 제어 파라미터
        self.kp = 0.045
        self.kd = 0.4
        self.max_force = 25.0
        
        # 시작 시간
        self.start_time = time.time()
        
        # 상태
        self.is_walking = False
        
        # GUI 슬라이더 생성
        self._create_sliders()
        
        # 카메라 설정
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.15]
        )
        
        print("=" * 60)
        print("Bounding Gait Simulation (강아지 달리기 모션)")
        print("-" * 60)
        print("Controls:")
        print("  - Step Length slider: 보폭 조절")
        print("  - Step Height slider: 발 들어올리는 높이")
        print("  - Height slider:      몸체 높이 조절")
        print("  - W key:              걷기 ON/OFF")
        print("  - R key:              로봇 리셋")
        print("  - Ctrl+C:             종료")
        print("-" * 60)
        print("보행 패턴: 앞다리(FL+FR) ↔ 뒷다리(RL+RR)")
        print("=" * 60)
    
    def _get_joint_names(self):
        """관절 이름과 ID 매핑"""
        joint_name_to_id = {}
        n_joints = p.getNumJoints(self.robot)
        
        for i in range(n_joints):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name = joint_info[1].decode('UTF-8')
            joint_name_to_id[joint_name] = joint_info[0]
            
        return joint_name_to_id
    
    def _create_sliders(self):
        """GUI 슬라이더 생성"""
        self.step_length_slider = p.addUserDebugParameter("Step Length", 0, 100, 60)
        self.step_height_slider = p.addUserDebugParameter("Step Height", 20, 80, 50)
        self.height_slider = p.addUserDebugParameter("Body Height", -30, 50, 20)
        
        # PD 게인 슬라이더
        self.kp_slider = p.addUserDebugParameter("Kp", 0, 0.1, self.kp)
        self.kd_slider = p.addUserDebugParameter("Kd", 0, 1.0, self.kd)
        self.force_slider = p.addUserDebugParameter("Max Force", 0, 50, self.max_force)
        
        # 보행 주기 슬라이더
        self.period_slider = p.addUserDebugParameter("Period (sec)", 0.5, 2.0, 1.0)
    
    def reset_robot(self):
        """로봇 위치 리셋"""
        p.resetBasePositionAndOrientation(
            self.robot,
            self.init_position,
            self.init_orientation
        )
        p.resetBaseVelocity(self.robot, [0, 0, 0], [0, 0, 0])
        self.start_time = time.time()
        print("[INFO] Robot position reset")
    
    def check_keyboard(self):
        """키보드 입력 확인"""
        keys = p.getKeyboardEvents()
        
        # R 키: 리셋
        if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
            self.reset_robot()
        
        # W 키: 걷기 토글
        if ord('w') in keys and keys[ord('w')] & p.KEY_WAS_TRIGGERED:
            self.is_walking = not self.is_walking
            status = "ON" if self.is_walking else "OFF"
            print(f"[INFO] Walking: {status}")
    
    def check_stability(self):
        """안정성 확인 - 넘어지면 리셋"""
        _, body_orn = p.getBasePositionAndOrientation(self.robot)
        euler = p.getEulerFromQuaternion(body_orn)
        
        # Roll 또는 Pitch가 45도 이상이면 리셋
        if abs(euler[0]) > math.pi/4 or abs(euler[1]) > math.pi/4:
            print("[WARNING] Robot fell over - Resetting...")
            self.reset_robot()
            return False
        return True
    
    def get_foot_positions(self, t):
        """
        시간 t에서의 4개 다리 발끝 위치 계산
        
        Parameters:
        -----------
        t : float
            현재 시간 (초)
        
        Returns:
        --------
        foot_positions : numpy array
            4개 다리의 발끝 위치
        """
        # 슬라이더 값 읽기
        step_length = p.readUserDebugParameter(self.step_length_slider)
        step_height = p.readUserDebugParameter(self.step_height_slider)
        period = p.readUserDebugParameter(self.period_slider)
        
        # BoundingGait 파라미터 업데이트
        self.bounding.period = period
        
        if self.is_walking:
            self.bounding.Sh = step_height
            kb_offset = {
                'IDstepLength': step_length,
                'IDstepWidth': 0.0,
                'IDstepAlpha': 0.0
            }
        else:
            # 정지 상태
            self.bounding.Sh = 0.0
            kb_offset = {
                'IDstepLength': 0.0,
                'IDstepWidth': 0.0,
                'IDstepAlpha': 0.0
            }
        
        # BoundingGait로 발끝 위치 계산
        foot_positions = self.bounding.positions(t, kb_offset)
        
        return foot_positions
    
    def apply_ik(self, foot_positions):
        """
        발끝 위치로 IK 계산 후 관절에 적용
        """
        # Body pose 파라미터
        height = p.readUserDebugParameter(self.height_slider)
        body_rot = (0, 0, 0)  # (roll, pitch, yaw)
        body_pos = (0, height, 0)  # (x, y, z)
        
        # IK 계산
        angles = self.kinematics.calcIK(foot_positions, body_rot, body_pos)
        
        # PD 게인 읽기
        kp = p.readUserDebugParameter(self.kp_slider)
        kd = p.readUserDebugParameter(self.kd_slider)
        max_force = p.readUserDebugParameter(self.force_slider)
        
        # 관절에 적용
        leg_names = ['front_left', 'front_right', 'rear_left', 'rear_right']
        part_names = ['shoulder', 'leg', 'foot']
        
        for leg_idx, leg_name in enumerate(leg_names):
            for part_idx, part_name in enumerate(part_names):
                joint_name = f"{leg_name}_{part_name}"
                
                if joint_name in self.joint_name_to_id:
                    joint_id = self.joint_name_to_id[joint_name]
                    target_angle = angles[leg_idx][part_idx] * self.dirs[leg_idx][part_idx]
                    
                    p.setJointMotorControl2(
                        bodyIndex=self.robot,
                        jointIndex=joint_id,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_angle,
                        positionGain=kp,
                        velocityGain=kd,
                        force=max_force
                    )
    
    def update_camera(self):
        """카메라가 로봇을 따라가도록 업데이트"""
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=robot_pos
        )
    
    def visualize_gait_phase(self, t):
        """보행 위상 시각화 (디버깅용)"""
        phase = (t % self.bounding.period) / self.bounding.period
        
        # 앞다리 위상
        front_phase = phase
        # 뒷다리 위상
        rear_phase = (phase + 0.5) % 1.0
        
        # 상태 텍스트
        front_state = "SWING" if front_phase < 0.5 else "STANCE"
        rear_state = "SWING" if rear_phase < 0.5 else "STANCE"
        
        return f"Front:{front_state} Rear:{rear_state}"
    
    def run(self):
        """메인 시뮬레이션 루프"""
        frame_count = 0
        
        try:
            while True:
                # 시간 계산
                t = time.time() - self.start_time
                
                # 키보드 입력 확인
                self.check_keyboard()
                
                # 안정성 확인
                self.check_stability()
                
                # 발끝 위치 계산
                foot_positions = self.get_foot_positions(t)
                
                # IK 적용
                self.apply_ik(foot_positions)
                
                # 카메라 업데이트
                self.update_camera()
                
                # 상태 표시 (1초마다)
                frame_count += 1
                if frame_count % 240 == 0:
                    robot_pos, _ = p.getBasePositionAndOrientation(self.robot)
                    distance = math.sqrt(robot_pos[0]**2 + robot_pos[1]**2)
                    status = "WALKING" if self.is_walking else "STOPPED"
                    gait_info = self.visualize_gait_phase(t)
                    
                    print(f"[{status}] Distance: {distance:.2f}m | "
                          f"Pos: ({robot_pos[0]:.2f}, {robot_pos[1]:.2f}) | "
                          f"Gait: {gait_info}")
                
                # 짧은 대기
                time.sleep(1./240.)
                
        except KeyboardInterrupt:
            print("\n[INFO] Simulation terminated by user")
        finally:
            p.disconnect()
            print("[INFO] PyBullet disconnected")


def main():
    """메인 함수"""
    sim = BoundingWalkingSimulation()
    sim.run()


if __name__ == "__main__":
    main()