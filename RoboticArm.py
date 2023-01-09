import time 
import RPi.GPIO as GPIO 
import numpy as np
from visual_kinematics.RobotSerial import RobotSerial
from visual_kinematics.Frame import Frame
from visual_kinematics.utility import simplify_angles


def initializeGPIO():
    GPIO.setmode(GPIO.BOARD)
    
def terminateGPIO():
    GPIO.cleanup()

class RoboticArm():
    angleAdjuster = lambda x : [round(np.rad2deg(i), 2) + 90 for i in x]
    def __init__(self, baseMotorPin, shoulderMotorPin, elbowMotorPin, wristMotorPin, DH_PARAMS, maximumIterations,PULSE_FREQ = 60):
       #set output pins
        GPIO.setup(baseMotorPin, GPIO.OUT)
        GPIO.setup(shoulderMotorPin, GPIO.OUT)
        GPIO.setup(elbowMotorPin, GPIO.OUT)
        GPIO.setup(wristMotorPin, GPIO.OUT)
        # set pins as PWM pins
        self.__baseMotor = GPIO.PWM(baseMotorPin, PULSE_FREQ) 
        self.__shoulderMotor = GPIO.PWM(shoulderMotorPin, PULSE_FREQ)
        self.__elbowMotor = GPIO.PWM(elbowMotorPin, PULSE_FREQ)
        self.__wristMotor = GPIO.PWM(wristMotorPin, PULSE_FREQ)
        dh_params = np.array(DH_PARAMS)
        self.robot = RobotSerial(dh_params=dh_params, max_iter=maximumIterations)
        return
    
    def reset(self):
        self.setBaseMotorAngle(90)
        self.setShoulderMotorAngle(90)
        self.setElbowMotorAngle(90)
        self.setWristMotorAngle(90)
    
    def inverseKinematics(self, x = 0, y = 0, z=0):
        xyz = np.array([[x], [y], [z]]) # translation matrix
        abc = np.array([0, 0, 0]) # rotation matrix
        end = Frame.from_euler_3(abc, xyz)
        self.robot.inverse(end)
        a1, a2, a3, a4 = RoboticArm.angleAdjuster(simplify_angles(self.robot.axis_values))
        print(a1, a2, a3, a4)
        result = False in [self.setBaseMotorAngle(a1), self.setShoulderMotorAngle(a2), self.setElbowMotorAngle(a3), self.setWristMotorAngle(a4)]
        if result == True:
            return None
        return a1, a2, a3, a4
      
    def forwardKinmatics(self, baseMotorAngle, shoulderMotorAngle, elbowMotorAngle, wristMotorAngle):
        self.setBaseMotorAngle(baseMotorAngle)
        self.setShoulderMotorAngle(shoulderMotorAngle)
        self.setElbowMotorAngle(elbowMotorAngle)
        self.setWristMotorAngle(wristMotorAngle)
        f = self.robot.forward(np.deg2rad([baseMotorAngle, shoulderMotorAngle, elbowMotorAngle, wristMotorAngle]))
        self.position = f.t_3_1.reshape([3, ])
        x, y, z = self.position
        return x , y , z
    
    def setBaseMotorAngle(self, angle):
        if angle < 0 or angle > 180:
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__baseMotor.start(0)
        self.__baseMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__baseMotor.stop()
 
        return True
    
    def setShoulderMotorAngle(self, angle):
        if angle < 90 or angle > 180:
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__shoulderMotor.start(0)
        self.__shoulderMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__shoulderMotor.stop()
        return True
    
    def setElbowMotorAngle(self, angle):
        if angle < 0 or angle > 180:
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__elbowMotor.start(0)
        self.__elbowMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__elbowMotor.stop()
        return True

    def setWristMotorAngle(self, angle):
        if angle < 0 or angle > 180:
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__wristMotor.start(0)
        self.__wristMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__elbowMotor.stop()
        return True


