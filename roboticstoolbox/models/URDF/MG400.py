#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.ERobot import ERobot, ERobot2
from spatialmath import SE3


class MG400(ERobot):
    """
    Class that imports a Panda URDF model

    ``Panda()`` is a class which imports a Franka-Emika Panda robot definition
    from a URDF file.  The model describes its kinematic and graphical
    characteristics.

    .. runblock:: pycon

        >>> import roboticstoolbox as rtb
        >>> robot = rtb.models.URDF.Panda()
        >>> print(robot)

    Defined joint configurations are:

    - qz, zero joint angle configuration, 'L' shaped configuration
    - qr, vertical 'READY' configuration
    - qs, arm is stretched out in the x-direction
    - qn, arm is at a nominal non-singular configuration

    .. codeauthor:: Jesse Haviland
    .. sectionauthor:: Peter Corke
    """

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "mg400_description/urdf/mg400.urdf"
        )

        print(links,name,urdf_filepath)
        
        super().__init__(
            links,
            name=name,
            manufacturer="Dobot",
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        # self.grippers[0].tool = SE3(0, 0, 0.1034)

        #[j1,j2,j3_1,j3,j4_1,j4,j2_2,j3_2,j4_2]
        j1= 0
        j2 = 1
        j3 = 1
        
        #[j1,j2,j3,   j3,j3,j3,j2,j3,j3]
        self.qr = np.array([j1,j2,j3,j3,-2*j3,j3,j2,j3,j3])
        self.qz = np.zeros(9)

        j1= 0
        j2 = 0.3
        j3 = 0.4
        self.qt = np.array([j1,j2,j3,0,-j2-j3,j3,j2,-j3,j3+j3])

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)
        self.addconfiguration("qt", self.qt)


if __name__ == "__main__":  # pragma nocover

    r = MG400()

    r.qz
