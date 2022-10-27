import pybullet as p
import pybullet_data
import time

# Environment setup
SAMPLING_RATE = 1e-2    # 0.01s = 10 ms
physics_client_id = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(SAMPLING_RATE)  # 1000Hz sampling rate
p.setGravity(0, 0, -9.81)
# p.setGravity(0, 0, 0)

# Setup plane
plane_id = p.loadURDF("plane.urdf")

# Setup objects
cubeStartPos = [0, 0, 1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)

# Perform simulation step
for i in range(5000):
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

# Exit Simulation
p.disconnect()
print("Simulation end")