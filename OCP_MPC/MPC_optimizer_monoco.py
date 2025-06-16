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

import os
import sys
import casadi as cs
from matplotlib.pyplot import get
import numpy as np
from copy import copy
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
from Utils.utils import skew_symmetric, v_dot_q, safe_mkdir_recursive, quaternion_inverse, euler_to_quaternion

class Monoco_Optimizer(object):
    def __init__(self, monoco_type, t_horizon=1, n_nodes=20,
                 q_cost=None, r_cost=None,
                 model_name="monoco_acados_mpc", solver_options=None): # insert one more argument here to show that I am using the ith iterated gp...
        
        """
        :param monoco_type: monoco typed object from (long_wing, short_wing, ultra-light_wing);
        :type monoco_type: monoco
        
        :param t_horizon: time horizon for MPC optimization
        :param n_nodes: number of optimization nodes until time horizon
        :param q_cost: diagonal of Q matrix for LQR cost of MPC cost function. Must be a numpy array of length 12.
        :param r_cost: diagonal of R matrix for LQR cost of MPC cost function. Must be a numpy array of length 3.
        :param solver_options: Optional set of extra options dictionary for solvers.
        """

        # Weighted squared error loss function q = (p_xyz, a_xyz, v_xyz, r_xyz), r = (u1, u2, u3, u4)
        if q_cost is None:
            q_cost = np.array([10, 10, 10, 0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        if r_cost is None:
            r_cost = np.array([0.1, 0.1, 0.1])

        self.T = t_horizon  # Time horizon needs to change 
        self.N = n_nodes  # number of control nodes within horizon - between initial time & time horizon, default value is 20 

        self.monoco = monoco_type

        self.max_u = monoco_type.max_input_value
        self.min_u = monoco_type.min_input_value

        # Declare model variables (row vectors with col vector 1)
        self.pos = cs.MX.sym('pos', 3)  # position
        self.ang = cs.MX.sym('ang', 3) # disk angles (rpy)
        # self.quat = cs.MX.sym('quat', 4)  # disk quaternions (wxyz)
        self.vel = cs.MX.sym('vel', 3)  # velocity
        self.bodyrate = cs.MX.sym('bodyrate', 3)  # disk body rates

        # Full state vector (12-dimensional)
        self.x = cs.vertcat(self.pos, self.ang, self.vel, self.bodyrate) # vertical concatenation
        self.state_dim = 12

        # Control input vector (rpy + collective thrust)
        u1 = cs.MX.sym('u1') # roll
        u2 = cs.MX.sym('u2') # pitch
        u3 = cs.MX.sym('u3') # collective thrust
        self.u = cs.vertcat(u1, u2, u3)

        # Disk dynamics
        self.monoco_nom_model = self.disk_dynamics() 
        self.monoco_nom_model = self.monoco_nom_model(x=self.x, u=self.u)

        # Setup acados model
        monoco_acados_model, nominal_monoco_dynamics = self.acados_setup_model(
            self.monoco_nom_model['x_dot'], model_name) # inputs of state and control input
        
        # Setup Acados OCP
        nx = monoco_acados_model.x.size()[0] # initial state - beginning of the series of nodes = 12
        nu = monoco_acados_model.u.size()[0] # initial inputs = 3
        ny = nx + nu
        n_param = monoco_acados_model.p.size()[0] if isinstance(monoco_acados_model.p, cs.MX) else 0

        # Create OCP object to formulate the optimization OCP = Optimal control problem
        ocp = AcadosOcp()
       
        ocp.model = monoco_acados_model# acados model is updated here...
        ocp.dims.N = self.N # time horizon / opt_dt
        ocp.solver_options.tf = t_horizon

        # Initialize parameters
        ocp.dims.np = n_param # 12
        ocp.parameter_values = np.zeros(n_param) # initialise parameters as zeros

        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # Formulate the LQR problem here ***********
        ocp.cost.W = np.diag(np.concatenate((q_cost, r_cost)))
        ocp.cost.W_e = np.diag(q_cost)
        terminal_cost = 0 if solver_options is None or not solver_options["terminal_cost"] else 1
        ocp.cost.W_e *= terminal_cost

        ocp.cost.Vx = np.zeros((ny, nx))
        ocp.cost.Vx[:nx, :nx] = np.eye(nx)
        ocp.cost.Vu = np.zeros((ny, nu))
        ocp.cost.Vu[-3:, -3:] = np.eye(nu)

        ocp.cost.Vx_e = np.eye(nx)

        # Initial reference trajectory (will be overwritten)
        x_ref = np.zeros(nx)
        ocp.cost.yref = np.concatenate((x_ref, np.array([0.0, 0.0, 0.0]))) # concatenates with control inputs
        ocp.cost.yref_e = x_ref # terminal shooting node 

        # Initial state constraint (will be overwritten) initial conditions can be updated here, set as constraint as ocp is firstly a problem object 
        ocp.constraints.x0 = x_ref

        # Set constraints
        ocp.constraints.lbu = np.array([self.min_u] * 3)
        ocp.constraints.ubu = np.array([self.max_u] * 3)
        ocp.constraints.idxbu = np.array([0, 1, 2])

        # Solver options
        ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.print_level = 0
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' if solver_options is None else solver_options["solver_type"]

        # Compile acados OCP solver if necessary
        self.acados_ocp_solver = AcadosOcpSolver(ocp, json_file=model_name + '_acados_ocp.json') # label and initialise the acadosolver here where ocp refers to the acados object



    def acados_setup_model(self, nominal, model_name):
        """
        Builds an Acados symbolic model using CasADi expressions. (via AcadosModel)
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

        # in case we want to add augmentations to the dynamics equations
        augmentations = cs.MX.sym('augmentations',12) 

        # nominal takes in inputs from supposed initial thruster values 
        dynamics_equations = nominal

        # state vector
        x_ = self.x

        # dynamics into the model
        dynamics_ = dynamics_equations

        acados_model = fill_in_acados_model(x=x_, u=self.u, p=augmentations, dynamics=dynamics_, name=model_name) # dict

        return acados_model, dynamics_equations # can recall dynamic equations 


    def disk_dynamics(self): # input for disk dynamics (x=self.x, u=self.u)
        """
        Symbolic disk dynamics. The state consists on: [p_xyz, a_wxyz, v_xyz, r_xyz]^T, where p
        stands for position, a for angle (in quaternion form), v for velocity and r for body rate. 
        
        The INPUT of the system is: [u_1, u_2, u_3]

        :return: CasADi function that computes the analytical differential state dynamics of the Monocopter's disk model.
        Inputs: 'x' state of Monocopter (12x1) and 'u' control input (4x1). Output: differential state vector 'x_dot'
        """
        
        x_dot = cs.vertcat(self.p_dot_dynamics(), self.ang_dot_dynamics(), self.v_dot_dynamics(), self.w_dot_dynamics()) # 12 x 1
        return cs.Function('x_dot', [self.x, self.u], [x_dot], ['x', 'u'], ['x_dot']) # function (name, args(input), args(output), name of args(input), name of args(output))


    def p_dot_dynamics(self): # returns velocity in W
        return self.vel


    def ang_dot_dynamics(self): # returns ang_rate in W
        return self.bodyrate


    def v_dot_dynamics(self): # dyn from uzh
        cyclic = self.u[0:2] * self.monoco.max_thrust_cyclic # max force value allocated
        collective = self.u[-1] * self.monoco.max_thrust_collective # max force value allocated
        g = cs.vertcat(0.0, 0.0, 9.81)
        quat = euler_to_quaternion(self.ang[0], self.ang[1], self.ang[2]) # from rpy, function from utils file, not data_process
        a_thrust = cs.vertcat(0.0, 0.0, cyclic[0] + cyclic[1] + collective[0]) / self.monoco.mass

        a_dynamics = v_dot_q(a_thrust, quat) - g # W frame 

        return a_dynamics


    def w_dot_dynamics(self):
        cyclic = self.u[0:2] * self.monoco.max_thrust_cyclic # max force value allocated

        return cs.vertcat(
            (cyclic[0] + (self.monoco.J[1] - self.monoco.J[2]) * self.bodyrate[1] * self.bodyrate[2]) / self.monoco.J[0],
            (cyclic[1] + (self.monoco.J[2] - self.monoco.J[0]) * self.bodyrate[2] * self.bodyrate[0]) / self.monoco.J[1],
            0.0)
    

    def set_reference_state(self, x_target=None, u_target=None): # set references here in this function 
        """
        Sets the target state and pre-computes the integration dynamics with cost equations
        :param x_target: 12-dimensional target state (p_xyz, ang_xyz, v_xyz, r_xyz)
        :param u_target: 3-dimensional target control input vector (u_1, u_2, u_3)
        """

        if x_target is None:
            x_target = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
        if u_target is None:
            u_target = [0, 0, 0]

        # Set new target state
        self.target = copy(x_target) # list

        ref = np.concatenate([x_target[i] for i in range(4)])
        ref = np.concatenate((ref, u_target)) # tries to minimise the control input

        # ref state/traj is updated here into the ocp solver where self.N is 20
        for j in range(self.N):
            self.acados_ocp_solver.set(j, "yref", ref)
        self.acados_ocp_solver.set(self.N, "yref", ref[:-3]) # terminal ref doesnt include control inputs

        # returns x_target + u_target
        return ref


    def run_optimization(self, initial_state=None):
        """
        Optimizes a trajectory to reach the pre-set target state, starting from the input initial state, that minimizes
        the quadratic cost function and respects the constraints of the system

        :param initial_state: 13-element list of the initial state. If None, 0 state will be used
        """

        if initial_state is None:
            initial_state = [0, 0, 0] + [0, 0, 0] + [0, 0, 0] + [0, 0, 0]

        # Set initial state. Add gp state if needed
        x_init = initial_state
        x_init = np.stack(x_init)

        ## Input initial states
        self.acados_ocp_solver.set(0, 'lbx', x_init) # lower bounds 
        self.acados_ocp_solver.set(0, 'ubx', x_init) # upper bounds 

        ## note, cannot predict using dynamic model cos mpc setup cant be iterated, only params can wo compiling..
        ## somehow, the update must be done externally either via p or x 
        
        ## how to change parameter P:
        #aug_state = [0.0, 0.0, 0.0] + [0.0, 0.0, 0.0, 0.0] + [v_b[0],v_b[1],v_b[2]] + [0.0, 0.0, 0.0]
        #aug_state = np.stack(aug_state)
        #self.acados_ocp_solver[use_model].set(0, 'p', aug_state) # aug_state taken from above    

        ## Solve OCP
        self.acados_ocp_solver.solve()

        # Get u
        w_opt_acados = np.ndarray((self.N, 3))
        x_opt_acados = np.ndarray((self.N + 1, len(x_init))) # N +1 due to terminal state
        x_opt_acados[0, :] = self.acados_ocp_solver.get(0, "x") # same dim and same values as initial state aka x_init
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_ocp_solver.get(i, "u") # roll, pitch, collective thrust 
            x_opt_acados[i + 1, :] = self.acados_ocp_solver.get(i + 1, "x") # states output - same dim as initial state but the back 3 are taken as body rates aka angular velocity

        w_opt_acados = np.reshape(w_opt_acados, (-1)) # first 3 values amount to roll, pitch, and thrust (only take this...)
        return (w_opt_acados, x_opt_acados)




