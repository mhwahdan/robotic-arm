"""
Pi Servo module.
"""
import time 

import RPi.GPIO as GPIO 

 
OUT_PIN = 11
PULSE_FREQ = 50

GPIO.setmode(GPIO.BOARD)
GPIO.setup(OUT_PIN, GPIO.OUT) 


def main():  
    print("Starting")
    servo1 = GPIO.PWM(OUT_PIN, PULSE_FREQ) 

    servo1.start(0) 

    print("Spinning")
    
    # Test the full range of movement. Note only integers are allowe
    servo1.ChangeDutyCycle(2)
    time.sleep(1)
    
    servo1.stop() 
    GPIO.cleanup()

if __name__ == "__main__":
    main()
