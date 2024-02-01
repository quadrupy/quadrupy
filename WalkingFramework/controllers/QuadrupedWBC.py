from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.solvers import MathematicalProgram, OsqpSolver, SolverOptions, CommonSolverOption
from pydrake.math import RotationMatrix
from pydrake.common.eigen_geometry import Quaternion, AngleAxis

import numpy as np

from .WalkingController import WalkingController, WorldToRobotCoordinates
from ..robots.WalkingRobot import WalkingRobot
from ..targets.WalkingTarget import WalkingTargetValue

class QuadrupedWBCSettings():
    def __init__(self,  des_height = 1,
                        step_time = 0.5,
                        step_height = 0.05,
                        raibert_params:dict = {'step_kd': [0.05,0.05],
                                               'step_clip': [0.18,0.12],
                                               'com_kd': [10.,20.]},
                        swing_params:dict = {'Kp': 20.,
                                             'Kd': 1.},
                        stance_params:dict = {'Kd': 1.},
                        wbqp_targets:dict = {'body_orientation': {'Kp': [100., 100., 0.],
                                                                  'Kd': [10., 10., 10.],
                                                                  'Kp_stand': [100., 100., 100.],
                                                                  'Kd_stand': [10., 10., 10.],
                                                                  'weights': [1.e+4,1.e+4,1.e+4]},
                                            'body_translation': {'Kp': [0., 0., 200.],
                                                                 'Kd': [0., 0., 20.],
                                                                 'Kp_stand': [100., 100., 200.],
                                                                 'Kd_stand': [10., 10., 20.],
                                                                 'weights': [1.e+4,1.e+4,1.e+4]}},
                        wbqp_params:dict = { 'mu': 0.5,
                                             'friction_cone_order': 8,
                                             'kin_constraint_gamma': 10.,
                                             'kin_constraint_dt': 0.1,
                                             'kin_constraint_ddx_max': 1.e+3,
                                             'actuation_cost': 1.,
                                             'contact_cost': 0.,
                                             'foot_on_ground_cost': 1.e+6,
                                             'kin_constraint_cost': 1.e+6,
                                             'reqularizing_cost': 1.e-6,
                                             'delta_cost': 1.e+1,
                                             'tau_filter_alpha': 0.5,
                                             'kb': 0.}):
        
        self.des_height = des_height
        self.step_time = step_time
        self.step_height = step_height
        self.raibert_params = raibert_params
        self.swing_params = swing_params
        self.stance_params = stance_params
        self.wbqp_targets = wbqp_targets
        self.wbqp_params = wbqp_params

    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('gait_params'):
            param_dict = config_dict['gait_params']
            if param_dict.__contains__('des_height'):
                self.des_height = param_dict['des_height']
            if param_dict.__contains__('step_time'):
                self.step_time = param_dict['step_time']
            if param_dict.__contains__('step_height'):
                self.step_height = param_dict['step_height']

        if config_dict.__contains__('raibert_params'):
            self.raibert_params = config_dict['raibert_params']
    
        if config_dict.__contains__('swing_params'):
            self.swing_params = config_dict['swing_params']

        if config_dict.__contains__('stance_params'):
            self.swing_params = config_dict['stance_params']

        if config_dict.__contains__('wbqp_targets'):
            self.wbqp_targets = config_dict['wbqp_targets']

        if config_dict.__contains__('wbqp_params'):
            self.wbqp_params = config_dict['wbqp_params']
            
        if config_dict.__contains__('gamepad'):
            self.gamepad = config_dict['gamepad']
        return

class QuadrupedWBC(WalkingController):
    def __init__(self,robot:WalkingRobot,settings:QuadrupedWBCSettings=None,config_dict:dict=None):
        WalkingController.__init__(self,robot.num_act)
        self.robot = robot
        self.controller_context = robot.plant.CreateDefaultContext()
        self.nq = robot.plant.num_positions()
        self.state_in = self.DeclareVectorInputPort('robot_state',robot.plant.num_multibody_states())

        self.settings = settings
        if self.settings is None:
            self.settings = QuadrupedWBCSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        self.leg_joint_indices = [self.robot.GetDependentJoints(c[0]) for c in self.robot.contacts] # List the joints that affect each contact point
        self.fsm = FiniteStateMachine(self.settings.step_time)
        self.swingfoot = SwingFootTrajectory(self.settings.step_time,self.settings.step_height)
        self.wbqp_two_contact = WholeBodyQP(robot,self.settings.wbqp_targets,self.settings.wbqp_params,n_contact=2)
        self.wbqp_four_contact = WholeBodyQP(robot,self.settings.wbqp_targets,self.settings.wbqp_params,n_contact=4)

        return

    def CalcOutput(self, context: Context, output: AbstractValue):
        # Get controller inputs
        t = context.get_time()
        qv_world = self.state_in.Eval(context)
        target_in:WalkingTargetValue = self.target_in.Eval(context)

        # Pull out some useful values
        xy_vel = qv_world[3:5]
        des_xy_vel = target_in.des_x_y_yaw_vel[:2]
        des_yaw_vel = target_in.des_x_y_yaw_vel[2]

        # Convert robot state to body coordinates
        if np.linalg.norm(qv_world) < 1e-6:
            return super().CalcOutput(context, output)
        qv_robot = WorldToRobotCoordinates(qv_world,self.nq)
        self.robot.plant.SetPositionsAndVelocities(self.controller_context,qv_robot)

        # Get current FSM state
        stance_feet, swing_feet, time_to_step = self.fsm.Update(t)

        # Get current foot positions and jacobians
        foot_locations = np.hstack([self.robot.plant.CalcPointsPositions(self.controller_context,c[0],c[1].flatten(),self.robot.plant.world_frame()) for c in self.robot.contacts]).T
        mean_stance_foot_x_y = np.average(foot_locations[stance_feet,:2],axis=0)
        foot_jacobians = [self.robot.plant.CalcJacobianTranslationalVelocity(self.controller_context,JacobianWrtVariable.kV,c[0],c[1].flatten(),self.robot.plant.world_frame(),self.robot.plant.world_frame()) for c in self.robot.contacts]
        foot_bias = [self.robot.plant.CalcBiasTranslationalAcceleration(self.controller_context,JacobianWrtVariable.kV,c[0],c[1].flatten(),self.robot.plant.world_frame(),self.robot.plant.world_frame()) for c in self.robot.contacts]

        # Switch between two and four contact whole body QP
        if len(swing_feet) == 0: # In quadruple stance
            # No desired swing foot acceleration
            des_sf_acc = None

            # Calculate the desired body accelerations
            des_body_acc = self.BodyPDControl(q_body = qv_robot[:7], v_body = qv_robot[self.nq:self.nq+6], des_x_y_pos=mean_stance_foot_x_y, des_x_y_yaw_vel=target_in.des_x_y_yaw_vel, use_standing_gains=True)

            # Get desired torques
            desired_tau = self.wbqp_four_contact.UpdateAndSolve(self.controller_context,\
                                                                stance_jacobians = [foot_jacobians[f] for f in stance_feet], stance_bias = [foot_bias[f] for f in stance_feet], des_body_acc  = des_body_acc,\
                                                                swing_jacobians  = [foot_jacobians[f] for f in swing_feet] , swing_bias  = [foot_bias[f] for f in swing_feet] , des_swing_acc = des_sf_acc)
        
        else: # In double stance
            # Get desired Raibert footsteps
            des_step_locations = self.CalcRaibertStepLocations(xy_vel,des_xy_vel,des_yaw_vel)

            # Get desired swing foot position, velocity and acceleration
            des_sf_pos, des_sf_vel, des_sf_acc_ff = self.swingfoot.Update(time_to_step,swing_feet,des_step_locations)

            # Get the pd acceleration for the whole body qp
            des_sf_acc = des_sf_acc_ff + self.settings.swing_params['Kp']*(des_sf_pos - foot_locations[swing_feet]) + self.settings.swing_params['Kd']*(des_sf_vel - foot_jacobians[swing_feet]@qv_robot[self.nq:])
            
            # Convert to joint positions and velocities for the pd controller
            des_swing_joint_pos, des_swing_joint_vel = self.SwingFootIK(swing_feet,des_sf_pos,des_sf_vel,foot_jacobians)

            # Calculate the desired body accelerations
            des_body_acc = self.BodyPDControl(qv_robot[6:], des_x_y_pos=mean_stance_foot_x_y, des_x_y_vel=target_in.des_x_y_yaw_vel, use_standing_gains=False)

            # Get desired torques
            desired_tau = self.wbqp_two_contact.UpdateAndSolve( self.controller_context,\
                                                                stance_jacobians = foot_jacobians[stance_feet], stance_bias = foot_bias[stance_feet], des_body_acc  = des_body_acc,\
                                                                swing_jacobians  = foot_jacobians[swing_feet] , swing_bias  = foot_bias[swing_feet] , des_swing_acc = des_sf_acc)
        
        # Fill in LLC output
        for i in stance_feet:
            idx = self.leg_joint_indices[i]
            self.actuation_data.q[idx] = 0.0
            self.actuation_data.Kp[idx] = 0.0
            self.actuation_data.dq[idx] = 0.0
            self.actuation_data.Kd[idx] = self.settings.stance_params['Kd']
            self.actuation_data.tau[idx] = desired_tau[idx]
        for i in swing_feet:
            idx = self.leg_joint_indices[i]
            self.actuation_data.q[idx] = des_swing_joint_pos[i]
            self.actuation_data.Kp[idx] = self.settings.swing_params['Kp']
            self.actuation_data.dq[idx] = des_swing_joint_vel[i]
            self.actuation_data.Kp[idx] = self.settings.swing_params['Kd']
            self.actuation_data.tau[idx] = desired_tau[idx]
        
        return super().CalcOutput(context, output)
    
    def CalcRaibertStepLocations(self,xy_vel,des_xy_vel,des_yaw_vel):
        kd = self.settings.raibert_params['step_kd']
        step_clip = self.settings.raibert_params['step_clip']
        step_time = self.settings.step_time
        des_height = self.settings.des_height
        g = 9.81
        v_cross_omega = np.array([xy_vel[1],-xy_vel[0]])*des_yaw_vel
        return np.clip(step_time/2*des_xy_vel + kd*(xy_vel - des_xy_vel) + des_height/g*v_cross_omega,-step_clip,step_clip)

    def SwingFootIK(self,swing_feet,des_sf_pos,des_sf_vel,J):
        des_swing_joint_pos = [self.robot.CalcFootIK(j,des_sf_pos[i]) for i,j in enumerate(swing_feet)]

        des_swing_joint_vel = []
        for i in swing_feet:         
            root_vel = self.robot.plant.GetVelocities(self.controller_context)
            root_vel[2:5] = 0.
            root_vel[6:] = 0.
            leg_j_idx = [j-6 for j in self.leg_joint_indices[i]]
            try:
                des_swing_joint_vel_ = np.linalg.inv(J[i][:,leg_j_idx])@(des_sf_vel - J@root_vel)
            except:
                des_swing_joint_vel_ = np.zeros(3)
            des_swing_joint_vel.append(des_swing_joint_vel_)

        return des_swing_joint_pos, des_swing_joint_vel

    def BodyPDControl(self,q_body,v_body,des_x_y_pos,des_x_y_yaw_vel,use_standing_gains=False):
        # Body tracking position targets
        des_body_pos = np.zeros(3)
        des_body_pos[:2] = des_x_y_pos
        des_body_pos[2] = self.settings.des_height
        des_body_vel = np.zeros(3)
        des_body_vel[:2] = des_x_y_yaw_vel[:2]
        # Body tracking rotation targets
        des_body_rot = np.array([1.0,0.0,0.0,0.0])
        des_body_rot_vel = np.zeros(3)
        des_body_rot_vel[2] = des_x_y_yaw_vel[2]

        # Body tracking gains
        if use_standing_gains:
            Kp_pos = self.settings.wbqp_targets['body_translation']['Kp_stand']
            Kd_pos = self.settings.wbqp_targets['body_translation']['Kd_stand']
            Kp_rot = self.settings.wbqp_targets['body_orientation']['Kp_stand']
            Kd_rot = self.settings.wbqp_targets['body_orientation']['Kd_stand']
        else:  
            Kp_pos = self.settings.wbqp_targets['body_translation']['Kp']
            Kd_pos = self.settings.wbqp_targets['body_translation']['Kd']
            Kp_rot = self.settings.wbqp_targets['body_orientation']['Kp']
            Kd_rot = self.settings.wbqp_targets['body_orientation']['Kd']

        # Calc desired accelerations
        des_body_acc = np.zeros(6)

        # Rotational PD
        body_rot_mat = RotationMatrix(Quaternion(q_body[:4]/np.linalg.norm(q_body[:4])))
        des_body_rot_mat = RotationMatrix(Quaternion(des_body_rot))
        body_rot_err:AngleAxis = (des_body_rot_mat@body_rot_mat.inverse()).ToAngleAxis()
        body_rot_err = body_rot_err.angle()*body_rot_err.axis()
        body_rot_vel_err = des_body_rot_vel - v_body[:3]
        des_body_acc[:3] = Kp_rot*body_rot_err + Kd_rot*body_rot_vel_err

        # Translational PD
        body_pos_err = des_body_pos - q_body[4:7]
        body_vel_err = des_body_vel - v_body[3:6]
        des_body_acc[3:] = Kp_pos*body_pos_err + Kd_pos*body_vel_err

        return des_body_acc
    
class WholeBodyQP():
    def __init__(self,robot:WalkingRobot,wbqp_targets,wbqp_params:dict,n_contact) -> None:
        self.robot = robot
        self.n_u = robot.num_act
        self.n_acc = robot.plant.num_velocities()
        self.n_contact = n_contact
        self.n_swing = robot.num_contacts - self.n_contact
        self.params = wbqp_params
        self.tau_prev = np.zeros(self.n_u)

        # Initialize the mathematical program
        self.prog = MathematicalProgram()
        self.joint_torques = self.prog.NewContinuousVariables(self.n_u,'tau')
        self.accelerations = self.prog.NewContinuousVariables(self.n_acc,'ddq')
        self.body_accelerations = self.accelerations[:6]
        self.joint_accelerations = self.accelerations[6:]
        self.contact_forces = np.vstack([self.prog.NewContinuousVariables(self.n_contact,'lambda_'+['x','y','z'][i]) for i in range(3)]).T # [[x_0,y_0,z_0],[x_1,y_1,z_1],...,[x_3,y_3,z_3]]
        self.contact_forces_vec = self.contact_forces.flatten() # [x_0,y_0,z_0,x_1,y_1,z_1,...,x_3,y_3,z_3]
        self.hardstop_slacks = self.prog.NewContinuousVariables(self.n_u,'s_hs')
        self.all_vars = self.prog.decision_variables()
        self.n_vars = len(self.all_vars)

        # Placeholder context for dynamics values
        ctx = robot.plant.CreateDefaultContext()

        # Initialize the dynamics values (J, dJdq, M, C, B)
        self.J_stance = np.zeros([3*self.n_contact,self.n_acc])
        self.dJdq_stance = np.zeros([3*self.n_contact,1])
        self.B = robot.plant.MakeActuationMatrix()
        self.CalcInertialDynamics(ctx)

        # Dynamics constraints (must be updated at each timestep)
        dyn_A,dyn_b = self.GetDynamicsConstraints()
        dyn_vars = np.hstack([self.accelerations,self.joint_torques,self.contact_forces_vec])
        self.dyn_constraint = self.prog.AddLinearEqualityConstraint(dyn_A,dyn_b,dyn_vars)
        self.dyn_constraint.evaluator().set_description('dynamics_constraint')

        # Joint hardstop constraints (must be updated at each timestep)
        # hs_A,hs_lb,hs_ub = self.GetJointHardstopConstraints(robot,ctx)
        # hs_vars = np.vstack([self.joint_accelerations,self.hardstop_slacks])
        # self.hs_constraint = self.prog.AddLinearConstraint(hs_A,hs_lb,hs_ub,hs_vars)
        # hs_slack_Q = np.eye(self.n_u)*self.params.get('kin_constraint_cost',1.e+6)
        # self.hs_slack_cost = self.prog.AddQuadraticCost(hs_slack_Q,np.zeros(self.n_u),self.hardstop_slacks)

        # Actuation constraints (unchanged across timesteps)
        self.torque_limits = np.vstack([-robot.actuation_limits,robot.actuation_limits]).T
        self.torque_constraint = self.prog.AddBoundingBoxConstraint(self.torque_limits[:,0],self.torque_limits[:,1],self.joint_torques)
        self.torque_constraint.evaluator().set_description('torque_constraint')

        # Friction cone constraints (unchanged across timesteps)
        fric_A,fric_lb,fric_ub = self.GetFrictionConeConstraints()
        self.fric_constraints = [self.prog.AddLinearConstraint(fric_A,fric_lb,fric_ub,self.contact_forces[i,:]) for i in range(self.n_contact)]
        [f.evaluator().set_description('fric_constraints_'+str(i)) for i,f in enumerate(self.fric_constraints)]

        # Foot on ground penalty (must be updated at each timestep)
        fog_Q,fog_b = self.GetFootOnGroundPenalty()
        self.fog_cost = self.prog.AddQuadraticCost(fog_Q,fog_b,self.accelerations,is_convex=True)
        self.fog_cost.evaluator().set_description('fog_cost')

        # Body tracking costs (must be updated at each timestep)
        self.body_tracking_Q = np.diag(np.concatenate([wbqp_targets['body_orientation']['weights'],wbqp_targets['body_translation']['weights']]))
        self.body_tracking_cost = self.prog.AddQuadraticCost(self.body_tracking_Q,np.zeros(6),self.body_accelerations)
        self.body_tracking_cost.evaluator().set_description('body_tracking_cost')

        if self.n_swing > 0:
            self.J_swing = np.zeros([3*self.n_swing,self.n_acc])
            self.dJdq_swing = np.zeros([3*self.n_swing,1])

            # Swing foot tracking costs (must be updated at each timestep)
            swing_Q,swing_b = self.GetSwingFootTrackingCosts(np.zeros(3*self.n_swing))
            self.swing_tracking_cost = self.prog.AddQuadraticCost(swing_Q,swing_b,self.accelerations)
            self.swing_tracking_cost.evaluator().set_description('swing_tracking_cost')

        # Actuation costs (unchanged across timesteps)
        actuation_Q = np.eye(self.n_u)*wbqp_params['actuation_cost']
        self.actuation_cost = self.prog.AddQuadraticCost(actuation_Q,np.zeros(self.n_u),self.joint_torques)
        self.actuation_cost.evaluator().set_description('actuation_cost')

        # Contact costs (unchanged across timesteps)
        contact_Q = np.eye(3*self.n_contact)*wbqp_params['contact_cost']
        self.contact_cost = self.prog.AddQuadraticCost(contact_Q,np.zeros(3*self.n_contact),self.contact_forces_vec)
        self.contact_cost.evaluator().set_description('contact_cost')

        # Regularizing costs (unchanged across timesteps)
        reg_Q = np.eye(self.n_vars)*wbqp_params['reqularizing_cost']
        self.reg_cost = self.prog.AddQuadraticCost(reg_Q,np.zeros(self.n_vars),self.all_vars)
        self.reg_cost.evaluator().set_description('reg_cost')

        self.solver = OsqpSolver()
        self.solver_options = SolverOptions()
        # self.solver_options.SetOption(CommonSolverOption.kPrintToConsole,1)
        # self.solver_options.SetOption(self.solver.solver_id(),'max_iter',10000)

        return

    def UpdateAndSolve(self,controller_context:Context,stance_jacobians,stance_bias,des_body_acc,swing_jacobians,swing_bias,des_swing_acc):
        # Update the dynamics and store the foot jacobians
        self.CalcInertialDynamics(controller_context)
        self.J_stance = np.vstack(stance_jacobians)
        self.dJdq_stance = np.vstack(stance_bias)

        # Update the mathematical program
        #   Dynamics constraint
        dyn_A,dyn_b = self.GetDynamicsConstraints()
        self.dyn_constraint.evaluator().UpdateCoefficients(dyn_A,dyn_b)

        #   Foot on ground penalty
        fog_Q,fog_b = self.GetFootOnGroundPenalty()
        self.fog_cost.evaluator().UpdateCoefficients(self.params['foot_on_ground_cost']*fog_Q,self.params['foot_on_ground_cost']*fog_b,is_convex=True)

        #   Swing foot tracking
        if self.n_swing > 0:
            # Update the foot jacobians
            self.J_swing = np.vstack(swing_jacobians)
            self.dJdq_swing = np.vstack(swing_bias)

            # Update the swing foot tracking cost
            swing_Q,swing_b = self.GetSwingFootTrackingCosts(des_swing_acc)
            self.swing_tracking_cost.evaluator().UpdateCoefficients(self.params['swing_foot_tracking_cost']*swing_Q,self.params['swing_foot_tracking_cost']*swing_b)

        #   Body tracking
        body_b = -self.body_tracking_Q@des_body_acc
        self.body_tracking_cost.evaluator().UpdateCoefficients(self.body_tracking_Q,body_b)

        # Solve the mathematical program
        # [c.evaluator().is_convex() for c in self.prog.GetAllCosts()]
        res = self.solver.Solve(self.prog,None,self.solver_options)
        
        # Check for succesful solve
        if res.is_success():
            # Get the desired torques
            tau_out = res.GetSolution(self.joint_torques)
            self.tau_prev = tau_out
        else:
            tau_out = self.tau_prev

        # Return the desired torques
        return tau_out

    def CalcInertialDynamics(self,ctx:Context):
        self.M = self.robot.plant.CalcMassMatrix(ctx)
        self.CG = self.robot.plant.CalcBiasTerm(ctx) - self.robot.plant.CalcGravityGeneralizedForces(ctx)

    def GetDynamicsConstraints(self):
        # M*ddq + C = B*tau + J^T*lambda
        # Variable order: [ddq,tau,lambda]
        return np.hstack([self.M,-self.B,-self.J_stance.T]), -self.CG

    def GetFrictionConeConstraints(self):
        # |lambda_x| + |lambda_y| <= mu*lambda_z
        # i.e.:
        #    lambda_x + lambda_y <= mu*lambda_z
        #    lambda_x - lambda_y <= mu*lambda_z
        #    -lambda_x + lambda_y <= mu*lambda_z
        #    -lambda_x - lambda_y <= mu*lambda_z
        # lambda_z >= 0
        # Variable order: [lambda_x,lambda_y,lambda_z]
        mu = self.params['mu']
        fric_A = np.array([[1.,1.,-mu],
                           [1.,-1.,-mu],
                           [-1.,1.,-mu],
                           [-1.,-1.,-mu],
                           [0.,0.,-1.]])
        fric_lb = -np.inf*np.ones(5)
        fric_ub = np.zeros(5)
        return fric_A, fric_lb, fric_ub

    def GetFootOnGroundPenalty(self):
        # Stance foot_acceleration = d(J*dq) = J*ddq + dJ*dq
        # Minimize 0.5*||J*ddq + dJ*dq||^2 = 0.5*ddq^T*J^T*J*ddq + (J^T*dJ*dq)^T*ddq + ... = 0.5*x^T*Q*x + b^T*x + ...
        # x = ddq, Q = J^T*J, b = J^T*dJ*dq
        return self.J_stance.T@self.J_stance, self.J_stance.T@self.dJdq_stance
        # return np.eye(self.n_acc), np.zeros(self.n_acc)

    def GetSwingFootTrackingCosts(self, des_swing_acc):
        # Swing foot acceleration = d(J*dq) = J*ddq + dJ*dq
        # Minimize 0.5*||J*ddq + dJ*dq - swing_acc||^2 = 0.5*ddq^T*J^T*J*ddq + (J^T*(dJ*dq-swing_acc))^T*ddq + ... = 0.5*x^T*Q*x + b^T*x + ...
        # x = ddq, Q = J^T*J, b = J^T*(dJ*dq - swing_acc)
        return self.J_swing.T@self.J_swing, self.J_swing.T@(self.dJdq_swing.flatten() - des_swing_acc.flatten())

    def GetJointHardstopConstraints(self,robot:WalkingRobot,ctx:Context):
        pass

    def DoubleIntegratorBarrierFunction(self,x,dx):
        gamma = self.params.get('kin_constraint_gamma',10.)
        dt = self.params.get('kin_constraint_dt',0.1)
        ddx_max = self.params.get('kin_constraint_ddx_max',1e3)

        h = x + dt*dx
        if h <= 0:
            return ddx_max 
        Lfh = dx
        Lgh = dt

        B = -np.log(h/(1+h))
        dBdh = -1/(h*(1+h))
        LfB = Lfh*dBdh
        LgB = Lgh*dBdh

        ddx_min = np.clip((gamma/B-LfB)/LgB,-ddx_max,ddx_max)

        return ddx_min


class FiniteStateMachine():
    def __init__(self,step_time) -> None:
        pass

    def Update(self,curr_time):
        stance_feet = [0,1,2,3]
        swing_feet = []
        time_to_step = np.inf

        return stance_feet, swing_feet, time_to_step

class SwingFootTrajectory():
    def __init__(self,step_time,step_height) -> None:
        pass

