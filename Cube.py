import pybullet as p
import pybullet_data
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# Create Cube
def create_cube(position, orientation, color):
    colliosion_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.1, 0.1, 0.1))
    visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.1, 0.1, 0.1), rgbaColor = color)
    return p.createMultiBody(1, colliosion_shape_id, visual_shape_id, position, orientation)

def decompose_homogenous_matrix(T):
    translation = T[:3, 3]
    rotation = T[:3, :3]
    quaternion = R.from_matrix(rotation).as_quat()
    return translation, quaternion

if __name__ == "__main__":
    #init gui
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    p.loadURDF("plane.urdf")
    initial_position = [0, 0, 0.1]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])

    create_cube(initial_position, initial_orientation, [1, 0, 0, 1])

    Rz = np.array([[np.cos(np.pi/4),     -np.sin(np.pi/4),   0],
                   [np.sin(np.pi/4),     np.cos(np.pi/4),    0],
                   [0,                    0,                   1]])
    
    Rx = np.array([ [1,                 0,                     0],
                    [0,                 np.cos(np.pi/6),     -np.sin(np.pi/6)],
                    [0,                 np.sin(np.pi/6),     np.cos(np.pi/6)]])
    
    Rc = Rx @ Rz


    T = np.array([[Rc[0, 0],        Rc[0, 1],           Rc[0, 2],   0.5 ],
                  [Rc[1, 0],        Rc[1, 1],           Rc[1, 2],    0.3 ],
                  [Rc[2, 0],        Rc[2, 1],           Rc[2, 2],    0.4   ],
                  [0,                0,                  0,          1   ]])
    
    new_position, new_orientation = decompose_homogenous_matrix(T)

    create_cube(new_position, new_orientation, [0, 1, 0, 1])    # new cube

    num_steps = 1000
    axis_length = 0.2

    for t in range(num_steps):
        p.addUserDebugLine(new_position, new_position + T[:3, 0]*axis_length, [1, 0, 0], lineWidth = 3, lifeTime = 1.0)
        p.addUserDebugLine(new_position, new_position + T[:3, 1]*axis_length, [0, 1, 0], lineWidth = 3, lifeTime = 1.0)
        p.addUserDebugLine(new_position, new_position + T[:3, 2]*axis_length, [0, 0, 1], lineWidth = 3, lifeTime = 1.0)
        p.stepSimulation()
        time.sleep(1)

    p.disconnect()