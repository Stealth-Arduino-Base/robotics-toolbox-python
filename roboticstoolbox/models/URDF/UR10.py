#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.ERobot import ERobot


class UR10(ERobot):
    """
    Class that imports a UR10 URDF model

    ``UR3()`` is a class which imports a Universal Robotics UR310 robot
    definition from a URDF file.  The model describes its kinematic and
    graphical characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.UR10()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "ur_description/urdf/ur10_joint_limited_robot.urdf.xacro"
        )

        super().__init__(
            links,
            name=name.upper(),
            manufacturer="Universal Robotics",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qr = np.array([np.pi, 0, 0, 0, np.pi / 2, 0])
        self.qz = np.zeros(6)
        self.rd = np.zeros(6)
        
        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        # reference pose, arm to the right, elbow up
        self.addconfiguration_attr(
            "rd", np.array([-0.0000, -0.8335, 0.0940, -3.1416, 0.8312, 3.1416])
        )

if __name__ == "__main__":  # pragma nocover

    robot = UR10()
    print(robot)
