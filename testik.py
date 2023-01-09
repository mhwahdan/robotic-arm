from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics import Robot
from visual_kinematics.Frame import Frame
import numpy as np
dh_params = np.array([[9, 0.,90, 0],
                      [0., 10, 0, 0],
                      [0., 12, 0, 0],
                      [0, 3, 0,0]])
robot = RobotSerial(dh_params)
theta = np.array([0., 0., 0, 0.])
f = robot.forward(theta)
xyz = np.array([[6], [8], [9]])
abc = np.array([0,0,0])
end = Frame.from_euler_3(abc, xyz)
a= robot.inverse(end)
print(a)