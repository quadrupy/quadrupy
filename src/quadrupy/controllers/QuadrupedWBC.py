from __future__ import annotations

from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.solvers import MathematicalProgram, OsqpSolver, SolverOptions
from pydrake.math import RotationMatrix, BsplineBasis, RollPitchYaw
from pydrake.trajectories import BsplineTrajectory
from pydrake.common.eigen_geometry import Quaternion, AngleAxis

import numpy as np
import time

from quadrupy.controllers.WalkingController import WalkingController, WorldToRobotCoordinates
from quadrupy.robots.WalkingRobot import WalkingRobot
from quadrupy.targets.WalkingTarget import WalkingTargetValue

class QuadrupedWBCSettings():
    def __init__(self,  des_height = 1,
                        step_time = 0.5,
                        step_height = 0.05,
                        raibert_params:dict = {'step_kd': [0.05,0.05],
                                               'step_clip': [0.18,0.12]},
                        swing_params:dict = {'Kp': 20.,
                                             'Kd': 1.},
                        stance_params:dict = {'Kd': 1.},
                        wbqp_targets_stand:dict = {'body_orientation': {'Kp': [100., 100., 100.],
                                                                        'Kd': [10., 10., 10.],
                                                                        'weights': [1.e+4,1.e+4,1.e+4]},
                                                    'body_translation':{'Kp': [100., 100., 200.],
                                                                        'Kd': [10., 10., 20.],
                                                                        'weights': [1.e+4,1.e+4,1.e+4]}},
                        wbqp_targets_walk:dict  = {'body_orientation': {'Kp': [100., 100., 0.],
                                                                        'Kd': [10., 10., 10.],
                                                                        'weights': [1.e+4,1.e+4,1.e+4]},
                                                    'body_translation': {'Kp': [0., 0., 200.],
                                                                        'Kd': [0., 0., 20.],
                                                                        'weights': [0.,0.,1.e+4]}},
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
        self.wbqp_targets_stand = wbqp_targets_stand
        self.wbqp_targets_walk = wbqp_targets_walk
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
            self.stance_params = config_dict['stance_params']

        if config_dict.__contains__('wbqp_targets_stand'):
            self.wbqp_targets_stand = config_dict['wbqp_targets_stand']

        if config_dict.__contains__('wbqp_targets_walk'):
            self.wbqp_targets_walk = config_dict['wbqp_targets_walk']

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
        self.swingfoot = SwingFootTrajectory(self.settings.step_height)
        self.wbqp_two_contact = WholeBodyQP(robot,self.settings.wbqp_targets_walk,self.settings.wbqp_params,n_contact=2)
        self.wbqp_four_contact = WholeBodyQP(robot,self.settings.wbqp_targets_stand,self.settings.wbqp_params,n_contact=4)

        self.killed = False
        self.t0 = time.time()

        return

    def CalcOutput(self, context: Context, output: AbstractValue):
        # Get controller inputs
        t = time.time() - self.t0
        qv_world = self.state_in.Eval(context)
        target_in:WalkingTargetValue = self.target_in.Eval(context)

        # Convert robot state to body coordinates
        if np.linalg.norm(qv_world) < 1e-6:
            # If the quaternion is zero, the robot is not initialized. Return zero torques
            return super().CalcOutput(context, output)
        robot_rot = RollPitchYaw(Quaternion(qv_world[:4]))
        # Pull out target values
        if abs(robot_rot.roll_angle())>np.pi/4 or abs(robot_rot.pitch_angle())>np.pi/4 or np.any(np.abs(qv_world[7:self.nq])>3) or target_in.kill:
            raise RuntimeError('Controller killed')
        des_xy_vel = target_in.des_x_y_yaw_vel[:2]
        des_yaw_vel = target_in.des_x_y_yaw_vel[2]
        start_stop = target_in.start_stop_stepping

        qv_robot = WorldToRobotCoordinates(qv_world,self.nq)
        self.robot.plant.SetPositionsAndVelocities(self.controller_context,qv_robot)

        # Get current FSM state
        stance_feet, swing_feet, time_to_step = self.fsm.Update(t, start_stop)

        # Get current foot positions and jacobians
        foot_locations = np.hstack([self.robot.plant.CalcPointsPositions(self.controller_context,c[0],c[1].flatten(),self.robot.plant.world_frame()) for c in self.robot.contacts]).T
        mean_stance_foot_x_y = np.average(foot_locations[stance_feet,:2],axis=0)
        foot_jacobians = [self.robot.plant.CalcJacobianTranslationalVelocity(self.controller_context,JacobianWrtVariable.kV,c[0],c[1].flatten(),self.robot.plant.world_frame(),self.robot.plant.world_frame()) for c in self.robot.contacts]
        foot_bias = [self.robot.plant.CalcBiasTranslationalAcceleration(self.controller_context,JacobianWrtVariable.kV,c[0],c[1].flatten(),self.robot.plant.world_frame(),self.robot.plant.world_frame()) for c in self.robot.contacts]

        # Get desired body position (centered between the feet). Note that when walking, the yaw is ignored
        des_x_y_yaw_pos = np.zeros(3)
        des_x_y_yaw_pos[:2] = mean_stance_foot_x_y

        # Switch between two and four contact whole body QP
        if len(swing_feet) == 0: # In quadruple stance
            # No desired swing foot acceleration
            des_sf_acc = None

            # Calculate the desired body accelerations
            foot_center_line = np.average(foot_locations[:2,:2] - foot_locations[2:,:2],axis=0)
            des_x_y_yaw_pos[2] = np.arctan2(foot_center_line[1],foot_center_line[0])
            des_body_acc = self.BodyPDControl(q_body = qv_robot[:7], v_body = qv_robot[self.nq:self.nq+6], des_x_y_yaw_pos=des_x_y_yaw_pos, des_x_y_yaw_vel=target_in.des_x_y_yaw_vel, use_standing_gains=True)

            # Get desired torques
            desired_tau = self.wbqp_four_contact.UpdateAndSolve(self.controller_context,\
                                                                stance_jacobians = [foot_jacobians[f] for f in stance_feet], stance_bias = [foot_bias[f] for f in stance_feet], des_body_acc  = des_body_acc,\
                                                                swing_jacobians  = [foot_jacobians[f] for f in swing_feet] , swing_bias  = [foot_bias[f] for f in swing_feet] , des_swing_acc = des_sf_acc)
        
        else: # In double stance
            # Get desired Raibert footsteps
            xy_vel = self.robot.plant.GetVelocities(self.controller_context)[3:5]
            des_step_location_single_foot = self.CalcRaibertStepLocations(xy_vel,des_xy_vel,des_yaw_vel)

            # Get desired swing foot position, velocity and acceleration pretending we only have one foot
            des_sf_pos_single_foot, des_sf_vel_single_foot, des_sf_acc_ff_single_foot = self.swingfoot.Update(np.average(foot_locations[swing_feet],axis=0),des_step_location_single_foot,time_to_step)

            # Map the single effective foot swing foot locations to the actual foot locations
            des_sf_pos, des_sf_vel, des_sf_acc_ff = self.MapToMultipleFeet(swing_feet,np.average(foot_locations[stance_feet,2]),des_sf_pos_single_foot, des_sf_vel_single_foot, des_sf_acc_ff_single_foot)

            # Get the pd acceleration for the whole body qp
            des_sf_acc = des_sf_acc_ff# + self.settings.swing_params['Kp']*(des_sf_pos - foot_locations[swing_feet]) + self.settings.swing_params['Kd']*(des_sf_vel - np.array([foot_jacobians[f]@qv_robot[self.nq:] for f in swing_feet]))
            
            # Convert to joint positions and velocities for the pd controller
            des_swing_joint_pos, des_swing_joint_vel = self.SwingFootIK(swing_feet,des_sf_pos,des_sf_vel,foot_jacobians)

            # Calculate the desired body accelerations
            des_body_acc = self.BodyPDControl(q_body = qv_robot[:7], v_body = qv_robot[self.nq:self.nq+6], des_x_y_yaw_pos=des_x_y_yaw_pos, des_x_y_yaw_vel=target_in.des_x_y_yaw_vel, use_standing_gains=False)

            # Get desired torques
            desired_tau = self.wbqp_two_contact.UpdateAndSolve(self.controller_context,\
                                                                stance_jacobians = [foot_jacobians[f] for f in stance_feet], stance_bias = [foot_bias[f] for f in stance_feet], des_body_acc  = des_body_acc,\
                                                                swing_jacobians  = [foot_jacobians[f] for f in swing_feet] , swing_bias  = [foot_bias[f] for f in swing_feet] , des_swing_acc = des_sf_acc)

        # Get anti-slip velocity targets for the stance feet
        des_stance_joint_vel = []
        for f in stance_feet:  
            root_vel = self.robot.plant.GetVelocities(self.controller_context)[:6]
            leg_j_idx = [j+6 for j in self.leg_joint_indices[f]]
            des_stance_joint_vel.append(-np.linalg.inv(foot_jacobians[f][:,leg_j_idx])@(foot_jacobians[f][:,:6]@root_vel))

        # Fill in LLC output
        for i,f in enumerate(stance_feet):
            idx = self.leg_joint_indices[f]
            self.actuation_data.q[idx] = 0.0
            self.actuation_data.Kp[idx] = 0.0
            self.actuation_data.dq[idx] = 0*des_stance_joint_vel[i]
            self.actuation_data.Kd[idx] = self.settings.stance_params['Kd']
            self.actuation_data.tau[idx] = desired_tau[idx]
        for i,f in enumerate(swing_feet):
            idx = self.leg_joint_indices[f]
            self.actuation_data.q[idx] = des_swing_joint_pos[i]
            self.actuation_data.Kp[idx] = self.settings.swing_params['Kp']
            self.actuation_data.dq[idx] = des_swing_joint_vel[i]
            self.actuation_data.Kd[idx] = self.settings.swing_params['Kd']
            self.actuation_data.tau[idx] = desired_tau[idx]
        
        return super().CalcOutput(context, output)
    
    def CalcRaibertStepLocations(self,xy_vel,des_xy_vel,des_yaw_vel):
        kd = np.array(self.settings.raibert_params['step_kd'])
        step_clip = np.array(self.settings.raibert_params['step_clip'])
        step_time = self.settings.step_time
        des_height = self.settings.des_height
        g = 9.81
        v_cross_omega = np.array([xy_vel[1],-xy_vel[0]])*des_yaw_vel
        return np.clip(step_time/2*des_xy_vel + kd*(xy_vel - des_xy_vel) + des_height/g*v_cross_omega,-step_clip,step_clip)

    def SwingFootIK(self,swing_feet,des_sf_pos,des_sf_vel,J):
        des_swing_joint_pos = []
        des_swing_joint_vel = []
        for i,f in enumerate(swing_feet): 
            # des_sf_pos_body_frame = self.robot.plant.CalcPointsPositions(self.controller_context,self.robot.plant.world_frame(),des_sf_pos[i],self.robot.root_frame).T[0]
            des_sf_pos_body_frame = des_sf_pos[i] - np.array([0.,0.,self.robot.plant.GetPositions(self.controller_context)[6]])
            des_swing_joint_pos.append(self.robot.CalcFootIK(f,des_sf_pos_body_frame))

            root_vel = self.robot.plant.GetVelocities(self.controller_context)[:6]
            root_vel[2:5] = 0.
            leg_j_idx = [j+6 for j in self.leg_joint_indices[f]]
            try:
                des_swing_joint_vel_ = np.linalg.inv(J[f][:,leg_j_idx])@(des_sf_vel[i] - J[f][:,:6]@root_vel)
            except:
                des_swing_joint_vel_ = np.zeros(3)
            des_swing_joint_vel.append(des_swing_joint_vel_)

        return des_swing_joint_pos, des_swing_joint_vel

    def MapToMultipleFeet(self,swing_feet,stance_foot_height,des_sf_pos_single_foot, des_sf_vel_single_foot, des_sf_acc_ff_single_foot):
        des_sf_pos = np.zeros([len(swing_feet),3])
        des_sf_vel = np.zeros([len(swing_feet),3])
        des_sf_acc_ff = np.zeros([len(swing_feet),3])
        for i,f in enumerate(swing_feet):
            des_sf_pos[i,:] = des_sf_pos_single_foot + self.robot.GetShoulderPosition(f)
            des_sf_vel[i,:] = des_sf_vel_single_foot
            des_sf_acc_ff[i,:] = des_sf_acc_ff_single_foot
        des_sf_pos[:,2] += stance_foot_height # offset the desired foot positions by the current stance foot height

        return des_sf_pos, des_sf_vel, des_sf_acc_ff

    def BodyPDControl(self,q_body,v_body,des_x_y_yaw_pos,des_x_y_yaw_vel,use_standing_gains=False):
        # Body tracking position targets
        des_body_pos = np.zeros(3)
        des_body_pos[:2] = des_x_y_yaw_pos[:2]
        des_body_pos[2] = self.settings.des_height
        des_body_vel = np.zeros(3)
        des_body_vel[:2] = des_x_y_yaw_vel[:2]
        # Body tracking rotation targets
        des_body_rot_mat = RotationMatrix(RollPitchYaw([0.,0.,des_x_y_yaw_pos[2]]))
        des_body_rot_vel = np.zeros(3)
        des_body_rot_vel[2] = des_x_y_yaw_vel[2]

        # Body tracking gains
        if use_standing_gains:
            Kp_pos = self.settings.wbqp_targets_stand['body_translation']['Kp']
            Kd_pos = self.settings.wbqp_targets_stand['body_translation']['Kd']
            Kp_rot = self.settings.wbqp_targets_stand['body_orientation']['Kp']
            Kd_rot = self.settings.wbqp_targets_stand['body_orientation']['Kd']
        else:  
            Kp_pos = self.settings.wbqp_targets_walk['body_translation']['Kp']
            Kd_pos = self.settings.wbqp_targets_walk['body_translation']['Kd']
            Kp_rot = self.settings.wbqp_targets_walk['body_orientation']['Kp']
            Kd_rot = self.settings.wbqp_targets_walk['body_orientation']['Kd']

        # Calc desired accelerations
        des_body_acc = np.zeros(6)

        # Rotational PD
        body_rot_mat = RotationMatrix(Quaternion(q_body[:4]/np.linalg.norm(q_body[:4])))
        body_rot_err:AngleAxis = (des_body_rot_mat@(body_rot_mat.inverse())).ToAngleAxis()
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
            self.swing_tracking_cost.evaluator().UpdateCoefficients(self.params['swing_foot_tracking_cost']*swing_Q,self.params['swing_foot_tracking_cost']*swing_b,is_convex=True)

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

    def GetSwingFootTrackingCosts(self, des_swing_acc: np.array):
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
        self.step_time = step_time
        self.phase = 0 # 0: standing, 1: left front and right rear stance, 2: right front and left rear stance
        self.stopping = False
        self.next_step_time = np.inf
        self.input_time = -np.inf

    def Update(self,curr_time,start_stop):
        if (curr_time - self.input_time) > 0.5 and start_stop:
            self.input_time = curr_time
            if self.phase == 0:
                self.phase = 1
                self.next_step_time = curr_time + self.step_time
            else:
                self.stopping = True # Can't switch to standing until the current phase is complete
            
        if (self.phase != 0) and (curr_time > self.next_step_time):
            if self.stopping:
                self.phase = 0
                self.next_step_time = np.inf
                self.stopping = False
            else:
                self.phase = self.phase%2 + 1
                self.next_step_time = curr_time + self.step_time

        if self.phase == 0:
            stance_feet = [0,1,2,3]
            swing_feet = []
            time_to_step = np.inf
        else:
            if self.phase == 1:
                stance_feet = [0,3]
                swing_feet = [1,2]
            else:
                stance_feet = [1,2]
                swing_feet = [0,3]
            time_to_step = self.next_step_time - curr_time

        return stance_feet, swing_feet, time_to_step

class SwingFootTrajectory():
    def __init__(self,step_height) -> None:
        self.horz_ind = [0,1]
        self.vert_ind = [2]
        self.time_to_touchdown_prev = -np.inf
        self.total_step_time = 0.25
        self.swf_y = np.zeros(3)
        self.swf_yd = np.zeros(3)
        self.swf_ydd = np.zeros(3)
        self.swf_height = step_height

    def Update(self,swf_pos,desired_pos,time_to_touchdown,touchdown_speed = 0.)->tuple[np.array,np.array,np.array]:
        if time_to_touchdown > self.time_to_touchdown_prev:
            # Just went through a phase transition
            self.swf_start = swf_pos
            self.total_step_time = time_to_touchdown
            self.reference_traj_z = build_z_spline(t0=-time_to_touchdown,tf=-0.0,x0=0*swf_pos[self.vert_ind],dx0=0,x1=self.swf_height,x2=-0.0,dx2=touchdown_speed)
        self.time_to_touchdown_prev = time_to_touchdown

        self.swf_y[self.vert_ind] = self.reference_traj_z.value(-time_to_touchdown)
        self.swf_yd[self.vert_ind] = self.reference_traj_z.EvalDerivative(-time_to_touchdown,1)
        self.swf_ydd[self.vert_ind] = self.reference_traj_z.EvalDerivative(-time_to_touchdown,2)
        
        reference_traj_xy = build_xy_spline(t0=-self.total_step_time,tf=0.0,xy0=self.swf_start[self.horz_ind],dxy0=np.zeros(len(self.horz_ind)),xy1=desired_pos,dxy1=np.array([0.,0.]))
        self.swf_y[self.horz_ind] = np.squeeze(reference_traj_xy.value(-time_to_touchdown))
        self.swf_yd[self.horz_ind] = np.squeeze(reference_traj_xy.EvalDerivative(-time_to_touchdown,1))
        self.swf_ydd[self.horz_ind] = np.squeeze(np.clip(reference_traj_xy.EvalDerivative(-time_to_touchdown,2),-10,10))

        return self.swf_y, self.swf_yd, self.swf_ydd
    
def build_z_spline(t0,tf,x0,dx0,x1,x2,dx2)->BsplineTrajectory:
    b = BsplineBasis(4,[t0,t0,t0,t0,(t0+tf)/2,tf,tf,tf,tf])
    c0 = x0
    c4 = x2
    c1 = x0 + dx0*(tf-t0)/6
    c3 = x2 - dx2*(tf-t0)/6
    c2 = 2*x1 - c1/2 - c3/2
    s = BsplineTrajectory(b,[[c0,c1,c2,c3,c4]])
    return s

def build_xy_spline(t0,tf,xy0,dxy0,xy1,dxy1)->BsplineTrajectory:
    b = BsplineBasis(5,[t0,t0,t0,t0,t0,tf,tf,tf,tf,tf])
    c0 = xy0
    c5 = xy1
    c1 = xy0 + dxy0*(tf-t0)/4
    c4 = xy1 - dxy1*(tf-t0)/4
    s = BsplineTrajectory(b,np.vstack((c0,c1,c4-(c5-c4),c4,c5)).T)
    return s