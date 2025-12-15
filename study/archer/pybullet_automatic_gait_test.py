"""
Simulation of SpotMicroAI and it's Kinematics 
Use a keyboard to see how it works
Use keyboard-Button to switch betweek walk on static-mode
"""
from os import system, name 
import sys
sys.path.append("..")

import matplotlib.animation as animation
import numpy as np
import time 
import math
import datetime as dt
import keyboard
import random

from environment import environment
import pybullet as p
import pybullet_data

import spotmicroai

from multiprocessing import Process
from Common.multiprocess_kb import KeyInterrupt

import kinematics as kn
from kinematicMotion import KinematicMotion, TrottingGait

rtime=time.time()
env=environment()

def reset():
    global rtime
    rtime=time.time()    

def resetPose():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz,joy_z
    joy_x, joy_y, joy_z, joy_rz = 128, 128, 128, 128

# define our clear function 
def consoleClear():

    # for windows 
    if name == 'nt': 
        _ = system('cls') 
  
    # for mac and linux(here, os.name is 'posix') 
    else: 
        _ = system('clear') 

robot=spotmicroai.Robot(False,True,reset)

spurWidth=robot.W/2+20
stepLength=0
stepHeight=72
iXf=120
iXb=-132

IDheight = p.addUserDebugParameter("height", -40, 90, 20)

Lp = np.array([[iXf, -100, spurWidth, 1], [iXf, -100, -spurWidth, 1],
[-50, -100, spurWidth, 1], [-50, -100, -spurWidth, 1]])

resetPose()
trotting=TrottingGait()

def smoothstep(x: float) -> float:
    # 0~1을 부드럽게 만들어주는 함수
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)

def lerp(a: float, b: float, s: float) -> float:
    return a * (1.0 - s) + b * s


POSE_T0 = time.time()


SIT_OFFSET = -55.0

STAND_OFFSET = 0.0

T_REST_HOLD   = 2.0
T_SIT_MOVE    = 2.0
T_SIT_HOLD    = 2.0
T_STAND_MOVE  = 2.0

def pose_offset_script(t: float) -> tuple[float, str]:
    """
    t(초)에 따라 (height_offset, label)을 반환
    REST(2s) -> SIT 내려가기(2s) -> SIT 유지(2s) -> STAND 올라가기(2s) -> STAND 유지
    """
    if t < T_REST_HOLD:
        return STAND_OFFSET, "REST"

    t2 = t - T_REST_HOLD
    if t2 < T_SIT_MOVE:
        s = smoothstep(t2 / T_SIT_MOVE)
        return lerp(STAND_OFFSET, SIT_OFFSET, s), "SIT(moving)"

    t3 = t2 - T_SIT_MOVE
    if t3 < T_SIT_HOLD:
        return SIT_OFFSET, "SIT(hold)"

    t4 = t3 - T_SIT_HOLD
    if t4 < T_STAND_MOVE:
        s = smoothstep(t4 / T_STAND_MOVE)
        return lerp(SIT_OFFSET, STAND_OFFSET, s), "STAND(moving)"

    return STAND_OFFSET, "STAND(hold)"
# =========================
# getJointState로 "중력 보상(모터가 버티는 토크)" 확인하기
# =========================
_last_tau_log_t = 0.0

def log_joint_torques_every_0_5s(label: str):
    global _last_tau_log_t
    now = time.time()
    if now - _last_tau_log_t < 0.5:
        return
    _last_tau_log_t = now

    # Robot 클래스에서 bodyUniqueId는 robot.quadruped 입니다.
    robot_id = robot.quadruped

    legs = ['front_left', 'front_right', 'rear_left', 'rear_right']
    parts = ['shoulder', 'leg', 'foot']

    taus = []
    for leg in legs:
        for part in parts:
            j = robot.jointNameToId[f"{leg}_{part}"]
            _, _, _, tau = p.getJointState(robot_id, j)
            taus.append(abs(tau))

    # 평균/최대 절대토크 출력 (값이 0이 아니면, 중력 때문에 모터가 실제로 힘을 쓰고 있다는 뜻입니다)
    print(f"[{label}] mean|tau|={np.mean(taus):.3f}, max|tau|={np.max(taus):.3f}")

def main(id, command_status):
    
    s=False

    while True:
        bodyPos=robot.getPos()
        bodyOrn,_,_=robot.getIMU()
        xr,yr,_= p.getEulerFromQuaternion(bodyOrn)
        distance=math.sqrt(bodyPos[0]**2+bodyPos[1]**2)

        if distance>50:
            robot.resetBody()
    
        ir=xr/(math.pi/180)
        
        d=time.time()-rtime

        base_h = p.readUserDebugParameter(IDheight)
        offset, pose_label = pose_offset_script(time.time() - POSE_T0)
        height = base_h + offset


        robot.feetPosition(Lp)

        roll=0
        robot.bodyRotation((roll, math.pi/180*((joy_x)-128)/3, -(1/256*joy_y-0.5)))

        bodyX=50+yr*10
        robot.bodyPosition((bodyX, 40+height, -ir))

        log_joint_torques_every_0_5s(pose_label)

        robot.step()
        consoleClear()


        robot.step()
        consoleClear()

if __name__ == "__main__":
    try:
        # Keyboard input Process
        KeyInputs = KeyInterrupt()
        KeyProcess = Process(target=KeyInputs.keyInterrupt, args=(1, KeyInputs.key_status, KeyInputs.command_status))
        KeyProcess.start()

        # Main Process 
        main(2, KeyInputs.command_status)
        
        print("terminate KeyBoard Input process")
        if KeyProcess.is_alive():
            KeyProcess.terminate()

    except Exception as e:
        print(e)
    finally:
        print("Done... :)")