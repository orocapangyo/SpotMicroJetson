import mujoco
import mujoco.viewer
import time

# MuJoCo 모델 파일
MODEL_PATH = "spotmicro.xml"

# 모델 불러오기
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

print("=== Week08 MuJoCo Simulation ===")
print("Simulation timestep:", model.opt.timestep)

# MuJoCo Viewer 실행
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        step_start = time.time()

        # 물리 시뮬레이션 진행
        mujoco.mj_step(model, data)

        # 화면 업데이트
        viewer.sync()

        # 실제 시간에 맞춰 실행
        elapsed = time.time() - step_start
        remaining = model.opt.timestep - elapsed

        if remaining > 0:
            time.sleep(remaining)