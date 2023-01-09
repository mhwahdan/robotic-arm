#!/usr/bin/env python3
# CLI for interacting with interop server.
from flask import Flask, jsonify
from requests import get
from math import pi

app = Flask(__name__)




@app.route('/')
def index():
    return "hello"

@app.route('/forward/<base>/<shoulder>/<elbow>/<wrist>/', methods= ["PUT", "GET"])
def forwardKinematics(base, shoulder, elbow, wrist):
    response = get("http://192.168.2.2:5000/forward/" + str(base) + "/" + str(shoulder) + "/" + str(elbow) + "/" + str(wrist) + "/")
    print(response)
    return jsonify(response.json())

@app.route('/inverse/<x>/<y>/<z>/', methods= ["PUT", "GET"])
def inverseKinematics(x, y, z):
    response = get("http://192.168.2.2:5000/inverse/" + str(x) + "/" + str(y) + "/" + str(z) + "/")
    print(response)
    return jsonify(response.json())


if __name__ == "__main__":
    app.run("0.0.0.0", debug=True)