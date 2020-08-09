import threading
import time
import numpy as np


def make_sim():
    import pybullet_data


    # OPTION 1 - NOT THREAD-SAFE
    # import pybullet as p
    # p.connect(p.DIRECT)

    # OPTION 2 - SEEMINGLY THREAD-SAFE
    import pybullet
    import pybullet_utils.bullet_client as bc
    p = bc.BulletClient(connection_mode=pybullet.DIRECT)


    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    plane = p.loadURDF("plane.urdf")
    sphereRadius = 0.05
    colSphereId = p.createCollisionShape(p.GEOM_SPHERE, radius=sphereRadius)
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE, rgbaColor=[1, 0, 1, 1], radius=sphereRadius)
    sphere = p.createMultiBody(0.1, colSphereId, visualShapeId, basePosition=[0, 0, sphereRadius + 0.00001])
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(0)
    p.applyExternalForce(sphere, -1, [100, 0, 0], posObj=[0, 0, 0], flags=p.LINK_FRAME)
    positions = []
    for _ in range(240):
        p.stepSimulation()
        positions.append(list(p.getBasePositionAndOrientation(sphere)[0]))

    return np.array(positions)


def worker(inst):
    pos = make_sim()
    np.savez(f"inst-{inst}.npz", positions=pos)


w1 = threading.Thread(target=worker, kwargs={"inst":0})
w2 = threading.Thread(target=worker, kwargs={"inst":1})
w3 = threading.Thread(target=worker, kwargs={"inst":2})

w1.start()
w2.start()
w3.start()

w1.join()
w2.join()
w3.join()

import matplotlib.pyplot as plt

x = np.arange(0,240)
data = []
for i in range(3):
    trajectory = np.load(f"inst-{i}.npz")["positions"]
    plt.plot(x, trajectory[:,0])

plt.show()

