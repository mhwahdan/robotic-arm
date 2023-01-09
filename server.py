#!/usr/bin/env python3
# CLI for interacting with interop server.
from flask import Flask, jsonify
import RoboticArm
from math import pi

app = Flask(__name__)




@app.route('/')
def index():
    return "hello"

@app.route('/forward/<base>/<shoulder>/<elbow>/<wrist>/', methods= ["PUT", "GET"])
def forwardKinematics(base, shoulder, elbow, wrist):
    RoboticArm.initializeGPIO()
    Robot  = RoboticArm.RoboticArm(baseMotorPin = 11, 
                               shoulderMotorPin = 13,
                               elbowMotorPin = 15,
                               wristMotorPin=16,
                               DH_PARAMS=
                               [[11.5, 0., pi / 2, 0],
                                [0., 10., 0, 0 ],
                                [0., 12.7, 0, 0],
                                [0, 2.8, 0, 0]],
                               maximumIterations=750)
    print(base, shoulder, elbow, wrist)
    x , y , z = Robot.forwardKinmatics(int(base), int(shoulder), int(elbow), int(wrist))
    response = [{
        "x": x,
        "y": y,
        "z": z
    }]
    RoboticArm.terminateGPIO()
    print(response)
    return jsonify(response)

@app.route('/inverse/<x>/<y>/<z>/', methods= ["PUT", "GET"])
def inverseKinematics(x, y, z):
    RoboticArm.initializeGPIO()
    Robot  = RoboticArm.RoboticArm(baseMotorPin = 11, 
                               shoulderMotorPin = 13,
                               elbowMotorPin = 15,
                               wristMotorPin=16,
                               DH_PARAMS=
                               [[11.5, 0., pi / 2, 0],
                                [0., 10., 0, 0 ],
                                [0., 12.7, 0, 0],
                                [0, 2.8, 0, 0]],
                               maximumIterations=750)
    print(x, y, z)
    res = Robot.inverseKinematics(float(x), float(y), float(z))
    if(res == None):
        RoboticArm.terminateGPIO()
        return ''
    base, shoulder, elbow, wrist = res
    response = [{
        "base": base,
        "shoulder": shoulder,
        "elbow": elbow,
        "wrist": wrist
    }]
    print(response)
    RoboticArm.terminateGPIO()
    
    return jsonify(response)


if __name__ == "__main__":
    app.run("0.0.0.0", debug=True)