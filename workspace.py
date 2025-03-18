import pybullet as p
import pybullet_data
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

def transformatin_matrix(tetha, translation):
    ct, st = np.cos(tetha), np.sin(tetha)
    tx = tz = translation
    return np.array([[ ct, 0, st, tx*st],
                     [  0, 1,  0,     0],
                     [-st, 0, ct, tz*ct],
                     [  0, 0, 0 ,     1]])

def compute_fk(joint_angles):
    L1, L2 = 1, 1
    theta1, tetha2 = joint_angles

    T_W_Base = np.eye(4)
    T_W_Base[2,3] = 0.05
    
    T1 = transformatin_matrix(theta1, L1)
    T_Link1 = T_W_Base @ T1

    T2 = transformatin_matrix(tetha2, L2)
    T_Link2 = T_Link1 @ T2

    return T_Link2[0,3], 0, T_Link2[2,3]


def draw_workspace():
    L1, L2 = 1, 1

    tetha1_range = np.linspace(0, 1/2*np.pi, 50)
    tetha2_range = np.linspace(0, 1/2*np.pi, 50)

    point = []
    color = []

    for tetha1 in tetha1_range:
        for tetha2 in tetha2_range:
            x, y, z = compute_fk([tetha1, tetha2])
            point.append([x, y, z])
            color.append([0,0,1])
    
    p.addUserDebugPoints(point, color, pointSize=2)

if __name__=="__main__":
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")

    robot_id = p.loadURDF("urdf/2dof_planar_robot.urdf", basePosition=[0,0,0], useFixedBase=True)
    time.sleep(1)



    joint_indices = [1, 2]
    joint_angles1 = np.linspace(0, 1/2*np.pi, 50)
    joint_angles2 = np.linspace(0, 1/2*np.pi, 50)

    draw_workspace()
    num_steps = 10000

    for _ in range(num_steps):
        for angle1 in joint_angles1:
            for angle2 in joint_angles2:
                angles = [angle1, angle2]
                for i , angle in zip(joint_indices, angles):
                    p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, targetPosition=angle)
                    p.stepSimulation()
                    time.sleep(0.1)
    p.disconnect()