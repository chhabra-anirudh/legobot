"""Tool orientation conventions shared by the planner and the reach map.

The tool z-axis points down. The fingers close along the tool's local Y, so zero
yaw grips along root Y and a quarter turn grips along root X.
"""
import numpy as np

DOWN = np.diag([1., -1., -1.])
GRIP_YAW = {'y': 0., 'x': np.pi/2}


def rotation_for(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ DOWN


ORIENTATIONS = {axis: rotation_for(yaw) for axis, yaw in GRIP_YAW.items()}
