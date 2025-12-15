import pybullet as p
import pybullet_data
import time

# PyBullet GUI 시작
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# 바닥과 간단한 로봇 로드
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("r2d2.urdf", [0, 0, 1])

# 시뮬레이션 실행
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()