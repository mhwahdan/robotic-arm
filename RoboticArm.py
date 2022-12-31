import time 
import RPi.GPIO as GPIO 
import tinyik
import numpy as np

def initializeGPIO():
    GPIO.setmode(GPIO.BOARD)
    
def terminateGPIO():
    GPIO.cleanup()

class RoboticArm():
    def __init__(self, baseMotorPin, shoulderMotorPin, elbowMotorPin, wristMotorPin, PULSE_FREQ = 50):
        GPIO.setup(baseMotorPin, GPIO.OUT)
        GPIO.setup(shoulderMotorPin, GPIO.OUT)
        GPIO.setup(elbowMotorPin, GPIO.OUT)
        GPIO.setup(wristMotorPin, GPIO.OUT)
        self.__baseMotor = GPIO.PWM(baseMotorPin, PULSE_FREQ) 
        self.__shoulderMotor = GPIO.PWM(shoulderMotorPin, PULSE_FREQ)
        self.__elbowMotor = GPIO.PWM(elbowMotorPin, PULSE_FREQ)
        self.__wristMotor = GPIO.PWM(wristMotorPin, PULSE_FREQ)
        self.Actuator = tinyik.Actuator(['x', [9., 0, 0], 'z', [10., 0, 0], 'z', [12., 0, 0], 'z', [3., 0, 0]])
        self.Actuator.angles = np.deg2rad([90, 45, 45, 90])
        self.position = self.Actuator.ee
        self.reset()
        return
    
    def reset(self):
        self.setBaseMotorAngle(90)
        self.setElbowMotorAngle(45)
        self.setShoulderMotorAngle(45)
        self.setElbowMotorAngle(45)
        self.setWristMotorAngle(90)
    
    def inverseKinematics(self, x = 0, y = 0):
          _x, _y, _z = self.Actuator.ee
          _x += (y * -1)
          _y += x
          self.Actuator.ee = _x, _y, _z
          angles =  np.round(np.rad2deg(self.Actuator.angles))
          print(angles)
          self.setBaseMotorAngle(angles[0])
          self.setShoulderMotorAngle(angles[1])
          self.setElbowMotorAngle(angles[2])
          self.setWristMotorAngle(angles[3])
          return angles
      
    def forwardKinmatics(self, baseMotorAngle, shoulderMotorAngle, elbowMotorAngle, wristMotorAngle):
        self.Actuator.angles = np.deg2rad([baseMotorAngle, shoulderMotorAngle, elbowMotorAngle])
        self.setBaseMotorAngle(self.Actuator.angles[0])
        self.setShoulderMotorAngle(self.Actuator.angles[1])
        self.setElbowMotorAngle(self.Actuator.angles[2])
        self.setElbowMotorAngle(self.Actuator.angles[3])
        return
    
    def setBaseMotorAngle(self, angle):
        if(angle > 180 or angle < 0):
            return False
        
        numberOfDegrees = float(angle) / 18 + 2
        self.__baseMotor.start(0)
        self.__baseMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__baseMotor.stop()
 
        return True
    
    def setShoulderMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = float(angle) / 20 + 3
        self.__shoulderMotor.start(0)
        self.__shoulderMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__shoulderMotor.stop()
        return True
    
    def setElbowMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__elbowMotor.start(0)
        self.__elbowMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__elbowMotor.stop()
        return True
    
    def setWristMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = float(angle) / 18 + 2
        self.__wristMotor.start(0)
        self.__wristMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        #self.__wristMotor.stop()
        return True         


