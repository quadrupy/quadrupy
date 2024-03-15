
from pydrake.systems.framework import LeafSystem, Context
from pydrake.common.value import AbstractValue

import numpy as np
import pygame

class WalkingTargetValue():
    def __init__(self) -> None:
        self.des_x_y_yaw_vel = np.zeros(3)
        self.start_stop_stepping = False
        self.kill = False

class WalkingTarget(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.target_value = WalkingTargetValue()
        self.target_value_out = self.DeclareAbstractOutputPort('target_value_out',lambda: AbstractValue.Make(WalkingTargetValue()),self.CalcOutput)

    def CalcOutput(self,context: Context,output: AbstractValue):
        output.set_value(self.target_value)
        return
    
class JoystickInputSettings():
    def __init__(self, x_y_yaw_offset = np.zeros(3), x_y_yaw_scale = np.ones(3), alpha_filt = 0.0):
        self.x_y_yaw_offset = x_y_yaw_offset
        self.x_y_yaw_scale = x_y_yaw_scale
        self.alpha_filt = alpha_filt

    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('x_y_yaw_offset'):
            self.x_y_yaw_offset = config_dict['x_y_yaw_offset']
        if config_dict.__contains__('x_y_yaw_scale'):
            self.x_y_yaw_scale = config_dict['x_y_yaw_scale']
        if config_dict.__contains__('alpha_filt'):
            self.alpha_filt = config_dict['alpha_filt']

class JoystickInput(WalkingTarget):
    def __init__(self, settings:JoystickInputSettings=None, config_dict:dict=None): 
        WalkingTarget.__init__(self)

        self.settings = settings
        if self.settings is None:
            self.settings = JoystickInputSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        pygame.init()
        if pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0)    
        else:    
            self.js = None

    def CalcOutput(self,context: Context,output: AbstractValue):
        if self.js is None and pygame.joystick.get_count() > 0:
            self.js = pygame.joystick.Joystick(0)
        if self.js is not None:
            pygame.event.get()
            if self.js.get_button(0):
                self.target_value.start_stop_stepping = True
            else:
                self.target_value.start_stop_stepping = False
            if self.js.get_button(1):
                self.target_value.kill = True

            des_x_y_vel_unfilt = np.multiply(self.settings.x_y_yaw_scale,np.array([self.js.get_axis(1),self.js.get_axis(0),self.js.get_axis(3)])) + self.settings.x_y_yaw_offset
            self.target_value.des_x_y_yaw_vel = self.settings.alpha_filt*self.target_value.des_x_y_yaw_vel + (1.-self.settings.alpha_filt)*des_x_y_vel_unfilt

        output.set_value(self.target_value)
        return