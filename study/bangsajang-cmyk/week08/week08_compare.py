import time
import mujoco
import pybullet as p


def test_mujoco(xml_path, steps=5000):

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    start = time.time()

    for _ in range(steps):
        mujoco.mj_step(model, data)

    elapsed = time.time() - start
    fps = steps / elapsed

    return elapsed, fps


def test_pybullet(steps=5000):

    p.connect(p.DIRECT)

    p.setGravity(0, 0, -9.81)

    # plane.urdf 없이 바닥 직접 생성
    plane_shape = p.createCollisionShape(
        p.GEOM_PLANE,
        planeNormal=[0, 0, 1]
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=plane_shape
    )

    start = time.time()

    for _ in range(steps):
        p.stepSimulation()

    elapsed = time.time() - start
    fps = steps / elapsed

    p.disconnect()

    return elapsed, fps


if __name__ == "__main__":

    steps = 5000
    XML_PATH = "../../../urdf/spot_micro.xml"

    print("=" * 50)
    print("Week08 PyBullet vs MuJoCo Performance Test")
    print("=" * 50)

    mujoco_time, mujoco_fps = test_mujoco(
        XML_PATH,
        steps
    )

    pybullet_time, pybullet_fps = test_pybullet(
        steps
    )

    print()
    print("[MuJoCo]")
    print(f"Execution Time : {mujoco_time:.4f} sec")
    print(f"Simulation FPS : {mujoco_fps:.2f}")

    print()
    print("[PyBullet]")
    print(f"Execution Time : {pybullet_time:.4f} sec")
    print(f"Simulation FPS : {pybullet_fps:.2f}")

    print()
    print("=" * 50)

    if mujoco_fps > pybullet_fps:
        print("Result : MuJoCo is faster")
    else:
        print("Result : PyBullet is faster")

    print("=" * 50)