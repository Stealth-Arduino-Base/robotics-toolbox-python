#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import roboticstoolbox as rp
import numpy as np

# Make a robot robot
robot = rp.models.MG400()

# Init joint to the 'ready' joint angles
robot.q = robot.qz

# Make 100 random sets of joint angles
q = np.zeros(6)*100


# Plot the joint trajectory with a 50ms delay between configurations
robot.plot(q=q, backend='pyplot', dt=1000)
# robot.plot(q=q, backend='swift', dt=0.050, vellipse=False, fellipse=False)
