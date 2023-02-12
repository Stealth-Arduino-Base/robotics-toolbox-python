#!/usr/bin/env python
"""
@author John Skinner
"""

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as sg
import numpy as np
import random 
import math
import time


# Create a robot in the default zero pose
robot = rtb.models.MG400()
robot.q = robot.qz

env = swift.Swift()
env.launch(realtime=True)
env.add(robot, robot_alpha=True, collision_alpha=False)

l_frame = sg.Axes(0.1)
l_frame.attach_to(robot.links[-1])
env.add(l_frame)


while True:
    
    Tep = sm.SE3.Rz(90*random.random(), unit='deg')*(robot.fkine(robot.qt)* sm.SE3.Trans(random.random()/10.0-0.02, 0,random.random()/10.0-0.08))
    sol = robot.ikine_LM(Tep)

    arrived = False
    dt = 0.05
    l_target = sg.Sphere(0.01, color=[0.2, 0.4, 0.65, 0.5], base=Tep)
    l_target_frame = sg.Axes(0.1, base=Tep)
    env.add(l_target)
    env.add(l_target_frame)
    robot.q = sol.q
    gain = [1,1,1,1,1,1]
    env.step(dt)
    print("residual = ", np.linalg.norm(Tep - robot.fkine(sol.q)))
    time.sleep(3)

# Uncomment to stop the browser tab from closing
# env.hold()
