#!/usr/bin/env python
"""
@author John Skinner
"""

import swift
import roboticstoolbox as rp
import numpy as np
import spatialgeometry as sg
import spatialmath as sm


# Create a mg400 in the default zero pose
mg400 = rp.models.MG400()
print(mg400)


env = swift.Swift()
env.launch(realtime=True)
mg400.q = mg400.qz
env.add(mg400, robot_alpha=True, collision_alpha=False)

dt = 0.05
interp_time = 5
wait_time = 2

poses = [ mg400.qz,mg400.qt,mg400.qz]

# lTep = (
#     sm.SE3.Tx(0.35)
#     * sm.SE3.Tz(0.3)
# )

# Pass through the reference poses one by one.
# This ignores the robot collisions, and may pass through itself
for previous, target in zip(poses[:-1], poses[1:]):
    for alpha in np.linspace(0.0, 1.0, int(interp_time / dt)):
        mg400.q = previous + alpha * (target - previous)
        
        
        lTep = mg400.fkine(mg400.q)
        l_frame = sg.Axes(0.1, pose= lTep)
        l_ft = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], pose=lTep)
        env.add(l_frame)
        env.add(l_ft)
        
        
        env.step(dt)
    for _ in range(int(wait_time / dt)):
        mg400.q = target
        env.step(dt)

# Uncomment to stop the browser tab from closing
env.hold()
