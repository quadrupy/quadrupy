from pydrake.common.value import AbstractValue
from pydrake.systems.framework import Context

import numpy as np

from .WalkingController import WalkingController, WorldToRobotCoordinates
from ..robots.WalkingRobot import WalkingRobot

class QuadrupedWBCSettings():
    def __init__(self,  des_height = 1,
                        step_time = 0.5,
                        step_height = 0.05,
                        raibert_params:dict = {'step_kd': [0.05,0.05],
                                               'step_clip': [0.18,0.12],
                                               'com_kd': [10.,20.]},
                        swing_params:dict = {'Kp': 20.,
                                             'Kd': 1.},
                        wbqp_targets:dict = {'body_orientation': {'Kp': [100., 100., 0.],
                                                                  'Kd': [10., 10., 10.],
                                                                  'weights': [1.e+4,1.e+4,1.e+4]},
                                            'body_translation': {'Kp': [0., 0., 200.],
                                                                 'Kd': [0., 0., 20.],
                                                                 'weights': [1.e+4,1.e+4,1.e+4]}},
                        wbqp_params:dict = { 'mu': 0.5,
                                             'friction_cone_order': 8,
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

        self.fsm = FiniteStateMachine(self.settings.step_time)
        self.swingfoot = SwingFootTrajectory(self.settings.step_time,self.settings.step_height)
        self.wbqp = WholeBodyQP(robot,self.settings.wbqp_targets,self.settings.wbqp_params)

        return

    def CalcOutput(self, context: Context, output: AbstractValue):
        # Get controller inputs
        t = context.get_time()
        qv_world = self.state_in.Eval(context)
        target_in = self.target_in.Eval(context)

        # Convert robot state to body coordinates
        if np.linalg.norm(qv_world) < 1e-6:
            return super().CalcOutput(context, output)
        qv_robot = WorldToRobotCoordinates(qv_world,self.nq)
        self.robot.plant.SetPositionsAndVelocities(self.controller_context,qv_robot)

        # Get current FSM state
        stance_feet, swing_feet, time_to_step = self.fsm.update(t)

        # Get current foot positions
        foot_locations = [self.robot.plant.CalcPointsPositions(self.controller_context,c[0],c[1].flatten(),self.robot.plant.world_frame()) for c in self.robot.contacts]

        if len(swing_feet) == 0:
            # In quadruple stance, no swing foot dynamics
            desired_body_position = np.average(foot_locations,axis=0)
            desired_body_position[2] = self.settings.des_height
        else:
            # In double stance, calculate swing foot positions
            # Get desired Raibert footsteps
            desired_step_location = self.settings.raibert_params['step_kd']

            # Get desired swing foot trajectories

        # Get desired stance leg torques

        # Fill in LLC output

        return super().CalcOutput(context, output)


        return
    
class WholeBodyQP():
    def __init__(self,robot,wbqp_targets,wbqp_params) -> None:
        pass

class FiniteStateMachine():
    def __init__(self,step_time) -> None:
        pass

    def update(self,curr_time):
        stance_feet = [0,1,2,3]
        swing_feet = []
        time_to_step = np.inf

        return stance_feet, swing_feet, time_to_step

class SwingFootTrajectory():
    def __init__(self,step_time,step_height) -> None:
        pass

