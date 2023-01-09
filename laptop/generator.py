from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from visual_kinematics.utility import simplify_angles
import numpy as np
from math import pi

robot = RobotSerial(dh_params=np.array([[11.5, 0., pi / 2, 0],
                                [0., 10., 0, 0 ],
                                [0., 12.7, 0, 0],
                                [0, 2.8, 0, 0]]),
                    max_iter=1000)

angleAdjuster = lambda x : [round(np.rad2deg(i), 2) + 90 for i in x]

def InverseKinematics(x, y, z):
    xyz = np.array([[x], [y], [z]])
    abc = np.array([0, 0, 0])
    end = Frame.from_euler_3(abc, xyz)
    robot.inverse(end)
    a1, a2, a3, a4 = angleAdjuster(simplify_angles(robot.axis_values))
    return a1, a2, a3, a4

theta = np.array([0., 0., 0., 0.])
f = robot.forward(theta)
#robot.show()
angles = InverseKinematics(0, -25, 9)
print(angles)
robot.show()
                