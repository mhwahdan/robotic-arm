import RoboticArm

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


RoboticArm.terminateGPIO()