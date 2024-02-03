from numpy.core.multiarray import array as array
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction
from pydrake.systems.framework import Context
from pydrake.common.value import AbstractValue

import numpy as np

from ..third_party.lib import inekf_py as inekf

from .WalkingObserver import WalkingObserver
from ..robots.WalkingRobot import WalkingRobot, SensorData

class InekfObserverSettings():
    def __init__(self, gyro_noise = 10., gyro_bias = 0.0, accel_noise = 10., accel_bias = 0.0, contact_noise = 10.):
        self.gyro_noise = gyro_noise
        self.gyro_bias = gyro_bias
        self.accel_noise = accel_noise
        self.accel_bias = accel_bias
        self.contact_noise = contact_noise
    
    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('gyro_noise'):
            self.gyro_noise = config_dict['gyro_noise']
        if config_dict.__contains__('gyro_bias'):
            self.gyro_bias = config_dict['gyro_bias']
        if config_dict.__contains__('accel_noise'):
            self.accel_noise = config_dict['accel_noise']
        if config_dict.__contains__('accel_bias'):
            self.accel_bias = config_dict['accel_bias']
        if config_dict.__contains__('contact_noise'):
            self.contact_noise = config_dict['contact_noise']

class InekfObserver(WalkingObserver):
    def __init__(self, robot:WalkingRobot, settings: InekfObserverSettings = None, config_dict: dict = None):
        WalkingObserver.__init__(self, robot.plant.num_actuators(), robot.num_contacts)
        self.robot = robot

        # Parse settings
        self.settings = settings
        if self.settings is None:
            self.settings = InekfObserverSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        # Initialize InEKF
        #   Noise parameters
        inekf_noise = inekf.NoiseParams()
        inekf_noise.setGyroscopeNoise(self.settings.gyro_noise)
        inekf_noise.setAccelerometerNoise(self.settings.accel_noise)
        inekf_noise.setGyroscopeBiasNoise(self.settings.gyro_bias)
        inekf_noise.setAccelerometerBiasNoise(self.settings.accel_bias)
        inekf_noise.setContactNoise(self.settings.contact_noise)
        #   Initial state (make sure to hold this in scope otherwise you will get a double free error)
        inekf_state = inekf.RobotState()
        inekf_state.setRotation(np.eye(3))
        inekf_state.setVelocity(np.zeros(3))
        inekf_state.setPosition(robot.plant.GetDefaultPositions()[4:7])
        inekf_state.setGyroscopeBias(np.zeros(3))
        inekf_state.setAccelerometerBias(np.zeros(3))
        self.inekf_state = inekf_state
        #   Kinematics object
        self.inekf_kinematics = [inekf.Kinematics(i,np.eye(4),np.eye(6)) for i in range(4)]
        #   Inekf filter
        self.inekf_filter = inekf.InEKF(inekf_state,inekf_noise)

    def CalcOutput(self, context: Context, output: AbstractValue):
        sensor_data:SensorData = self.sensing_in.Eval(context)
        
        super().CalcOutput(context, output)
        return

