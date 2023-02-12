#!/usr/bin/env python
"""
@author Jesse Haviland
"""

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np
import random 
import math

env = swift.Swift()
env.launch(realtime=True)

# env.set_camera_pose([1.4, 0, 0.7], [0, 0.0, 0.5])

r = rtb.models.MG400()
env.add(r)

r.q = r.qt

Tep = r.fkine(r.q)
l_target = sg.Sphere(0.01, color=[0.2, 0.2, 0.2, 0.2], base=Tep)
l_target_frame = sg.Axes(0.1, base=Tep)
env.add(l_target)
env.add(l_target_frame)




l_frame = sg.Axes(0.1)
l_frame.attach_to(r.links[-1])
env.add(l_frame)

# # Construct an ETS for the left and right arms
# la = r.ets(end=r.links[-1])


while True:
    
    Tep = sm.SE3.Rz(90*random.random(), unit='deg')*(r.fkine(r.qt)* sm.SE3.Trans(random.random()/10.0-0.02, 0,random.random()/10.0-0.08))

    arrived = False
    dt = 0.05
    l_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep)
    l_target_frame = sg.Axes(0.1, base=Tep)
    env.add(l_target)
    env.add(l_target_frame)
    
    gain = [1,1,1,1,1,1]
    while not arrived:
        v, arrived= rtb.p_servo(r.fkine(r.q), wTep= Tep,method="angle-axis", threshold=0.01)
        r.qd = np.linalg.pinv(r.jacobe(r.q)) @ v
        env.step(dt)


print("destination reached")
env.hold()
