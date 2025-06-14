import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

guiFlag = False

dt = 1/240

kp = 200.0
ki = 140.0
kd = 50.0

g = 10
L = 0.8
L1 = L
L2 = L

# target_x1 = 0.7
# target_y1 = -0.4
# target_x2 = 1.5
# target_y2 = -0.4

# Target angles, rad
target_th1 = 0.8
target_th2 = -0.5

# Target angles
# target_th1 = np.arcsin(target_x1/ L1)
# target_th2 = np.arcsin((target_x2 - target_x1)/ L1)


physicsClient = p.connect(p.GUI if guiFlag else p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -g)
boxId = p.loadURDF("./two-link.urdf.xml", useFixedBase=True)

p.changeDynamics(boxId, 1, linearDamping=0, angularDamping=0)
p.changeDynamics(boxId, 2, linearDamping=0, angularDamping=0)

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.POSITION_CONTROL)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=3, controlMode=p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

p.setJointMotorControl2(bodyIndex=boxId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(bodyIndex=boxId, jointIndex=3, controlMode=p.VELOCITY_CONTROL, force=0)


maxTime = 5
logTime = np.arange(0, maxTime, dt)
sz = len(logTime)
log_th1 = np.zeros(sz)
log_th2 = np.zeros(sz)


int_err1 = 0.0
int_err2 = 0.0

prev_err1 = 0.0
prev_err2 = 0.0

idx = 0

for t in logTime:
    th1, dth1 = p.getJointState(boxId, 1)[:2]
    th2, dth2 = p.getJointState(boxId, 3)[:2]

    # first joint
    err1 = target_th1 - th1
    int_err1 += err1 * dt
    d_err1 = (err1 - prev_err1) / dt
    torque1 = kp * err1 + ki * int_err1 + kd * d_err1
    prev_err1 = err1

    # second joint
    err2 = target_th2 - th2
    int_err2 += err2 * dt
    d_err2 = (err2 - prev_err2) / dt
    torque2 = kp * err2 + ki * int_err2 + kd * d_err2
    prev_err2 = err2

    if idx == 0:
        torque1 = kp * err1
        torque2 = kp * err2

    p.setJointMotorControlArray(
        bodyIndex=boxId,
        jointIndices=[1, 3],
        controlMode=p.TORQUE_CONTROL,
        forces=[torque1, torque2]
    )

    p.stepSimulation()

    log_th1[idx] = th1
    log_th2[idx] = th2

    idx += 1

    if guiFlag:
        time.sleep(dt)

p.disconnect()

plt.subplot(2, 1, 1)
plt.title("target th1 = "+"%.3f"%target_th1 + " rad")
plt.plot(logTime, log_th1, label='th1')
plt.plot(logTime, np.ones_like(logTime)*target_th1, '--', label='target th1')
plt.legend()

plt.subplot(2, 1, 2)
plt.title("target th2 = "+"%.3f"%target_th2 + " rad")
plt.plot(logTime, log_th2, label='th2')
plt.plot(logTime, np.ones_like(logTime)*target_th2, '--', label='target th2')
plt.legend()

plt.tight_layout()
plt.show()
