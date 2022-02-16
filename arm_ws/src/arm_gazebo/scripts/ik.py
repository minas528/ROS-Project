import rospy
import tinyik as ik
from arm_lib.srv import IK, IKResponse


def inverse_kinematics(req):
    arr = [
        [0, 0, 0.1],
        "z", [0, 0, 0.05],
        "x", [0, 0, 2.0],
        "x", [0, 0, 1.0],
        "x", [0, 0, .5],
        "z", [0, 0, 0.04],
        "x", [0, 0, 0.04],
        [0, 0, 0.2]
    ]
    arm = ik.Actuator(arr)
    arm.ee = req.end_effector
    return IKResponse(arm.angles)


def init_server():
    rospy.init_node('reverse_kinematics_server')
    service = rospy.Service('ik', IK, inverse_kinematics)
    print("server has started")
    service.spin()


if __name__ == "__main__":
    init_server()
