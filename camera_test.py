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
boxId = p.loadURDF("panda.urdf", cubeStartPos, cubeStartOrientation)

# Camera setting
# Get depth values using the OpenGL renderer
width = 128
height = 128
view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[1, -1, 1.6],
                                                  distance=0.2,
                                                  yaw=40,
                                                  pitch=-40,
                                                  roll=0,
                                                  upAxisIndex=2)
projection_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=width / height, nearVal=0.01, farVal=20)

# Perform simulation step
for i in range(5000):
    p.stepSimulation()
    time.sleep(SAMPLING_RATE)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    images = p.getCameraImage(width,
                              height,
                              view_matrix,
                              projection_matrix,
                              shadow=True,
                              renderer=p.ER_BULLET_HARDWARE_OPENGL)

# Exit Simulation
p.disconnect()
print("Simulation end")