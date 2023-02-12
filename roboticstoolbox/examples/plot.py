#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import roboticstoolbox as rp
import numpy as np

# Make a panda robot
panda = rp.models.MG400()

# Init joint to the 'ready' joint angles
panda.q = panda.qz

# Make 100 random sets of joint angles
arr = np.random.rand(100, 4)


rows = 5
cols = 4

arr[:, 0] = 0
arr[:, -1] = - arr[:, 1] - arr[:, 2]

# Plot the joint trajectory with a 50ms delay between configurations
panda.plot(q=arr, backend='pyplot', dt=1)
# panda.plot(q=q, backend='swift', dt=0.050, vellipse=False, fellipse=False)
