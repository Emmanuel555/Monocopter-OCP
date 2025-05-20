""" Implementation of the nonlinear optimizer for the Monocopter.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>."""

import casadi as cs
from matplotlib.pyplot import get
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from Utils.utils import skew_symmetric, v_dot_q, safe_mkdir_recursive, quaternion_inverse

class Monoco_Optimizer:
    def __init__(self, monoco_type, t_horizon=1, n_nodes=20,
                 q_cost=None, r_cost=None,
                 model_name="augmented_quad_3d_acados_mpc", solver_options=None): # insert one more argument here to show that I am using the ith iterated gp...
        """
        :param quad: monoco typed object from (long_wing, short_wing, ultra-light_wing);
        :type quad: monoco
        
        :param t_horizon: time horizon for MPC optimization
        :param n_nodes: number of optimization nodes until time horizon
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 12.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 4.
        :param B_x: dictionary of matrices that maps the outputs of the gp regressors to the state space.
        
        :param q_mask: Optional boolean mask that determines which variables from the state compute towards the cost
        function. In case no argument is passed, all variables are weighted.
        
        :param solver_options: Optional set of extra options dictionary for solvers.
        """

        # Weighted squared error loss function q = (p_xyz, a_xyz, v_xyz, r_xyz), r = (u1, u2, u3, u4)
        if q_cost is None:
            q_cost = np.array([10, 10, 10, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        if r_cost is None:
            r_cost = np.array([0.1, 0.1, 0.1, 0.1])

        self.T = t_horizon  # Time horizon needs to change 
        self.N = n_nodes  # number of control nodes within horizon - between initial time & time horizon, default value is 20 

        self.monoco = monoco_type

        self.max_u = monoco_type.max_input_value
        self.min_u = monoco_type.min_input_value

        # Declare model variables
        self.p = cs.MX.sym('p', 3)  # position
        self.q = cs.MX.sym('a', 4)  # angle quaternion (wxyz)
        self.v = cs.MX.sym('v', 3)  # velocity
        self.r = cs.MX.sym('r', 3)  # angle rate

        # Full state vector (13-dimensional)
        self.x = cs.vertcat(self.p, self.q, self.v, self.r) # horizontal concatenation
        self.state_dim = 13

        # Control input vector (rpy + collective thrust)
        u1 = cs.MX.sym('u1')
        u2 = cs.MX.sym('u2')
        u3 = cs.MX.sym('u3')
        u4 = cs.MX.sym('u4')
        self.u = cs.vertcat(u1, u2, u3, u4)


    def acados_setup_model(self, nominal, model_name):
        """
        Builds an Acados symbolic models using CasADi expressions.
        :arg model_name: name for the acados model for the relevant Monocopter type.
        :arg nominal: CasADi symbolic nominal model of the Monocopter: f(self.x, self.u) = x_dot, dimensions 13x1.
        :return:
            - An AcadosModel of the Monocopter 
            - CasADi symbolic nominal dynamics equations (long,short,ultra-light)
        :rtype: AcadosModel, cs.MX
        """
    
        def fill_in_acados_model(x, u, p, dynamics, name):

            x_dot = cs.MX.sym('x_dot', dynamics.shape)
            f_impl = x_dot - dynamics # taken from pendulum example

            # Define Dynamics model
            model = AcadosModel()
            model.f_expl_expr = dynamics
            model.f_impl_expr = f_impl
            model.x = x
            model.xdot = x_dot # standard x dot in x dot = ax + bu
            model.u = u
            model.p = p
            model.name = name

            return model

        acados_models = {}
        dynamics_equations = {}

        # Run GP inference if GP's available

        # the entire section below needs to be updated in the optimisation section at the bottom
        """ if self.gp_reg_ensemble is not None:

            # Feature vector are the elements of x and u determined by the selection matrix B_z. The trigger var is used
            # to select the gp-specific state estimate in the first optimization node, and the regular integrated state
            # in the rest. The computing of the features z is done within the GPEnsemble.
            gp_x = self.gp_x * self.trigger_var + self.x * (1 - self.trigger_var)
            #  Transform velocity to body frame
            v_b = v_dot_q(gp_x[7:10], quaternion_inverse(gp_x[3:7]))
            gp_x = cs.vertcat(gp_x[:7], v_b, gp_x[10:])
            gp_u = self.u

            gp_dims = self.gp_reg_ensemble.dim_idx

            # Get number of models in GP
            for i in range(self.gp_reg_ensemble.n_models):
                # Evaluate cluster of the GP ensemble
                cluster_id = {k: [v] for k, v in zip(gp_dims, i * np.ones_like(gp_dims, dtype=int))}
                outs = self.gp_reg_ensemble.predict(gp_x, gp_u, return_cov=False, gp_idx=cluster_id, return_z=False)

                # Unpack prediction outputs. Transform back to world reference frame
                outs = self.add_missing_states(outs)
                gp_means = v_dot_q(outs["pred"], gp_x[3:7])
                gp_means = self.remove_extra_states(gp_means)

                print ("Analysing GP mean size: ", gp_means.shape)
                
                # Jus to look at the dimensions
                print ("B.x is: ", self.B_x)
                print ("cs.mtimes(self.B_x, gp_means) is... ", cs.mtimes(self.B_x, gp_means))

                # Add GP mean prediction - nominal refers to nominal dynamic model 
                dynamics_equations[i] = nominal + cs.mtimes(self.B_x, gp_means) # need to amend this part out

                x_ = self.x
                dynamics_ = dynamics_equations[i]

                # Add again the gp augmented dynamics for the GP state
                dynamics_ = cs.vertcat(dynamics_)
                dynamics_equations[i] = cs.vertcat(dynamics_equations[i])

                i_name = model_name + "_domain_" + str(i)

                params = cs.vertcat(self.gp_x, self.trigger_var)
                acados_models[i] = fill_in_acados_model(x=x_, u=self.u, p=params, dynamics=dynamics_, name=i_name)

        else: """

        # No available GP so return nominal dynamics
        # start with nominal dynamics - this part needs to edit first
        # TO DO - find out actual dynamics of nominal model 
        # nominal dynamics = 13 x 1 matrix 

        augmentations = cs.MX.sym('augmentations',13) # for now, this matrix aug of 13 x 1, lets jus assume its true...

        # nominal takes in inputs from supposed initial thruster values 

        dynamics_equations[0] = nominal + augmentations

        x_ = self.x
        #dynamics_ = nominal

        # test this
        dynamics_ = dynamics_equations[0]

        acados_models[0] = fill_in_acados_model(x=x_, u=self.u, p=augmentations, dynamics=dynamics_, name=model_name) # dict

        return acados_models, dynamics_equations # can recall dynamic equations 


    def quad_dynamics(self, rdrv_d): # input for quad dynamics (rdrv_d = drag coeff)
        """
        Symbolic dynamics of the 3D quadrotor model. The state consists on: [p_xyz, a_wxyz, v_xyz, r_xyz]^T, where p
        stands for position, a for angle (in quaternion form), v for velocity and r for body rate. 
        
        The INPUT of the system is: [u_1, u_2, u_3, u_4], i.e. the activation of the four thrusters.

        :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
        by Faessler et al.

        :return: CasADi function that computes the analytical differential state dynamics of the quadrotor model.
        Inputs: 'x' state of quadrotor (13x1) and 'u' control input (4x1). Output: differential state vector 'x_dot'
        (13x1)
        """

        x_dot = cs.vertcat(self.p_dynamics(), self.q_dynamics(), self.v_dynamics(rdrv_d), self.w_dynamics()) # 13 x 13
        return cs.Function('x_dot', [self.x[:13], self.u], [x_dot], ['x', 'u'], ['x_dot']) # function (name, args, function/output, [options])

    def p_dynamics(self): # returns velocity
        return self.v

    def q_dynamics(self): 
        return 1 / 2 * cs.mtimes(skew_symmetric(self.r), self.q)

    def v_dynamics(self, rdrv_d):
        """
        :param rdrv_d: a 3x3 diagonal matrix containing the D matrix coefficients for a linear drag model as proposed
        by Faessler et al. None, if no linear compensation is to be used.
        """

        f_thrust = self.u * self.quad.max_thrust
        g = cs.vertcat(0.0, 0.0, 9.81)
        a_thrust = cs.vertcat(0.0, 0.0, f_thrust[0] + f_thrust[1] + f_thrust[2] + f_thrust[3]) / self.quad.mass

        v_dynamics = v_dot_q(a_thrust, self.q) - g # assumes that self.q is currently in quaternion form thats why need to change it to matrix form 

        if rdrv_d is not None:
            # Velocity in body frame:
            v_b = v_dot_q(self.v, quaternion_inverse(self.q))
            rdrv_drag = v_dot_q(cs.mtimes(rdrv_d, v_b), self.q)
            v_dynamics += rdrv_drag

        #rdrv_drag = np.zeros((3, 3), float)
        #np.fill_diagonal(rdrv_drag, [0.1, 0.1, 0.1])
        rdrv_drag = np.array([0.0,0.0,0.0]) # shape is 3,
        #v_b = v_dot_q(self.v, quaternion_inverse(self.q))
        #rdrv_drag = v_dot_q(cs.mtimes(rdrv, v_b), self.q)
        rdrv_drag = cs.MX(rdrv_drag)
        v_dynamics += rdrv_drag

        return v_dynamics

    def w_dynamics(self):
        f_thrust = self.u * self.quad.max_thrust

        y_f = cs.MX(self.quad.y_f)
        x_f = cs.MX(self.quad.x_f)
        c_f = cs.MX(self.quad.z_l_tau)
        return cs.vertcat(
            (cs.mtimes(f_thrust.T, y_f) + (self.quad.J[1] - self.quad.J[2]) * self.r[1] * self.r[2]) / self.quad.J[0],
            (-cs.mtimes(f_thrust.T, x_f) + (self.quad.J[2] - self.quad.J[0]) * self.r[2] * self.r[0]) / self.quad.J[1],
            (cs.mtimes(f_thrust.T, c_f) + (self.quad.J[0] - self.quad.J[1]) * self.r[0] * self.r[1]) / self.quad.J[2])



