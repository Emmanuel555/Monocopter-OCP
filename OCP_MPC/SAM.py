#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#
from casadi import SX, vertcat, sin, cos
from math import sqrt
import numpy as np
from Utils.utils import quaternion_to_euler, skew_symmetric, v_dot_q, unit_quat, quaternion_inverse


class SAM(object):
    def __init__(self, monoco_name):
        """
        Initialization of the monocopter class   
        """

        # Crazyflie Max PWM input
        self.cf_max = 65500

        # Max cyclic & collective PWM inputs 
        self.max_thrust_cyclic = 10000/self.cf_max
        self.max_thrust_collective = 55500/self.cf_max 

        # Max & min PWM inputs for the motor 
        self.max_input_value = 65500/self.cf_max  # Motors at full thrust = 1
        self.min_input_value = -65500/self.cf_max  # Motors turned off = -1

        # Monoco name 
        self.monoco_name = monoco_name

        # Monocopter's intrinsic parameters as a disk
        if monoco_name == 'short':
            self.J = np.array([1/10000,1/10000,0.0])  # N m s^2 = kg m^2
            self.mass = 0.0064  # kg
        elif monoco_name == 'long':
            self.J = np.array([.03, .03, .06])  # N m s^2 = kg m^2
            self.mass = 0.058  # kg
        elif monoco_name == 'ultralight':
            self.J = np.array([.03, .03, .06])  # N m s^2 = kg m^2
            self.mass = 0.032  # kg



        


    