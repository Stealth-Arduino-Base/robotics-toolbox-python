#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np

env = swift.Swift()
env.launch(realtime=True)

# env.set_camera_pose([1.4, 0, 0.7], [0, 0.0, 0.5])

r = rtb.models.MG400()
env.add(r)

lTep = (r.fkine(r.qz))


l_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=lTep)
l_target_frame = sg.Axes(0.1, base=lTep)
env.add(l_target)
env.add(l_target_frame)


# l_frame = sg.Axes(0.1, pose=r.grippers[0].tool)
# l_ft = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=r.grippers[0].tool)
# env.add(l_frame)
# env.add(l_ft)


# Construct an ETS for the left and right arms
la = r.ets(end=r.grippers[0])

arrivedl = False
arrivedr = False

dt = 0.05

gain = np.array([1, 1, 1, 1.6, 1.6, 1.6])

while not arrivedl or not arrivedr:

    vl, arrivedl = rtb.p_servo(la.fkine(r.q), lTep, gain, threshold=1)

    r.qd[la.jindices] = np.linalg.pinv(la.jacobe(r.q)) @ vl

    env.step(dt)


print("destination reached")
env.hold()
