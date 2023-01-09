import tinyik
import numpy as np

leg = tinyik.Actuator(['x', [9., 0, 0], 'z', [10., 0, 0], 'z', [12., 0, 0]])
leg.angles = np.deg2rad([90, 45, 45])
print(leg.ee)
tinyik.visualize(leg)