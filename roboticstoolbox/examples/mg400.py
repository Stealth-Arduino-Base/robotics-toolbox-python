#!/usr/bin/env python
"""
@author John Skinner
"""

import swift
import roboticstoolbox as rp
import numpy as np

env = swift.Swift()
env.launch(realtime=True)

# Create a mg400 in the default zero pose
mg400 = rp.models.MG400()
print(mg400)
mg400.q = mg400.qz
env.add(mg400, robot_alpha=True, collision_alpha=False)

dt = 0.05
interp_time = 5
wait_time = 2

poses = [ mg400.qz,mg400.qt]

# Pass through the reference poses one by one.
# This ignores the robot collisions, and may pass through itself
for previous, target in zip(poses[:-1], poses[1:]):
    for alpha in np.linspace(0.0, 1.0, int(interp_time / dt)):
        mg400.q = previous + alpha * (target - previous)
        env.step(dt)
    for _ in range(int(wait_time / dt)):
        mg400.q = target
        env.step(dt)

# Uncomment to stop the browser tab from closing
env.hold()
