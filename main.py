import RoboticArm
import time 

'''
Base Motor Pin => 11

Shoulder Motor Pin => 13

Elbow Motor Pin => 15

Wrist Motor => 16
'''

RoboticArm.initializeGPIO()

Robot  = RoboticArm.RoboticArm(baseMotorPin = 11, 
                               shoulderMotorPin = 13,
                               elbowMotorPin = 15,
                               wristMotorPin = 16)                                                          
while True:
    Robot.setBaseMotorAngle(0)
    Robot.setShoulderMotorAngle(0)
    Robot.setElbowMotorAngle(0)
    Robot.setWristMotorAngle(0)
    time.sleep(1)
    Robot.setBaseMotorAngle(90)
    Robot.setShoulderMotorAngle(90)
    Robot.setElbowMotorAngle(90)
    Robot.setWristMotorAngle(90)
    time.sleep(1)


RoboticArm.terminateGPIO()