import rospy
import numpy as np
from arm_lib.srv import FK, FKResponse


def rotation_x(rad):
    return np.array([
        [1, 0, 0, 0],
        [0, np.cos(rad), -np.sin(rad), 0],
        [0, np.sin(rad), +np.cos(rad), 0],
        [0, 0, 0, 1]
    ])


def rotation_y(rad):
    return np.array([
        [np.cos(rad), -np.sin(rad), 0, 0],
        [np.sin(rad), +np.cos(rad), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def rotation_z(rad):
    return np.array([
        [+np.cos(rad), 0, np.sin(rad), 0],
        [0, 1, 0, 0],
        [-np.sin(rad), 0, np.cos(rad), 0],
        [0, 0, 0, 1]
    ])


def translate(distance, axis):
    convert = {0: (0, 3), 1: (1, 3), 2: (2, 3)}
    m = np.eye(4)
    m[convert[axis]] = distance
    return m


def forward_kinematics(req):
    convert = {0: rotation_x, 1: rotation_z, 2: rotation_y}

    res = translate(0.1, 2)
    for i in range(0, len(req.link_length)):
        res = res.dot(convert[req.joint_axis[i]](req.joint_position[i]).dot(translate(req.link_length[i], 2)))
    return FKResponse(res[:, 3])


def init_server():
    rospy.init_node('forward_kinematics_server')
    service = rospy.Service('fk', FK, forward_kinematics)
    print("server has started")
    service.spin()


if __name__ == "__main__":
    init_server()
