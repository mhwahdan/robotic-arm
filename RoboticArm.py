import time 
import RPi.GPIO as GPIO 


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
        #Center all motors on 
        self.__baseMotor.start(0)
        self.__baseMotor.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.__baseMotor.stop()
        self.__shoulderMotor.start(0)
        self.__shoulderMotor.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.__shoulderMotor.stop()
        self.__elbowMotor.start(0)
        self.__elbowMotor.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.__elbowMotor.stop()
        self.__wristMotor.start(0)        
        self.__wristMotor.ChangeDutyCycle(7)
        time.sleep(0.5)
        self.__wristMotor.stop()
        return
        
    def setBaseMotorAngle(self, angle):
        angle += 90
        if(angle > 180 or angle < 0):
            return False
        
        numberOfDegrees = int(angle / 18) + 2
        self.__baseMotor.start(0)
        self.__baseMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        self.__baseMotor.stop()
        return True
    
    def setShoulderMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = int(angle / 20) + 3
        self.__shoulderMotor.start(0)
        self.__shoulderMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        self.__shoulderMotor.stop()
        return True
    
    def setElbowMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = int(angle / 18) + 2
        self.__elbowMotor.start(0)
        self.__elbowMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        self.__elbowMotor.stop()
        return True
    
    def setWristMotorAngle(self, angle):
        if(angle > 90 or angle < 0):
            return False
        numberOfDegrees = int(angle / 18) + 2
        self.__wristMotor.start(0)
        self.__wristMotor.ChangeDutyCycle(numberOfDegrees)
        time.sleep(1)
        self.__wristMotor.stop()
        return True         


