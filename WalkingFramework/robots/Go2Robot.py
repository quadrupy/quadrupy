from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform
from pydrake.geometry import HalfSpace, Box

import numpy as np
import os

from .WalkingRobot import WalkingRobot

class Go2RobotSettings():
    def __init__(self, friction = 0.5, llc_dt = 0.001, controller_dt = 0.01):
        self.friction = friction
        self.llc_dt = llc_dt                    # The time step of the low level controllers (on the Go1 hardware this is fixed at 1e-3)
        self.controller_dt = controller_dt      # The time step of the high level controller, for walking 1e-2 is a good rate
    
    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('friction'):
            self.friction = config_dict['friction']
        if config_dict.__contains__('llc_dt'):
            self.llc_dt = config_dict['llc_dt']
        if config_dict.__contains__('controller_dt'):
            self.controller_dt = config_dict['controller_dt']

class Go2Robot(WalkingRobot):
    def __init__(self, is_sim=True, settings: Go2RobotSettings = None, config_dict: dict = None):
        self.is_sim = is_sim

        self.settings = settings
        if self.settings is None:
            self.settings = Go2RobotSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        builder = DiagramBuilder()
        plant: MultibodyPlant
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=self.settings.llc_dt)
        parser = Parser(plant)
        parser.AddModels(os.path.join(os.path.dirname(__file__), 'assets/Go2/go2_description.urdf'))

        # Drake doesn't import actuators correctly from the URDF, so we have to do it manually
        joint_actuation_limits = {'hip':23.7,'thigh':23.7,'calf':45.43} # These values come from the urdf
        actuation_limits = []
        ji = plant.GetJointIndices(plant.GetModelInstanceByName("go2_description"))
        for i in ji:
            j = plant.get_joint(i)
            if j.num_velocities() > 0:
                plant.AddJointActuator(j.name(),j)
                act_lim = np.inf
                for jtype in joint_actuation_limits.keys():
                    if jtype in j.name(): 
                        act_lim = joint_actuation_limits[jtype]
                actuation_limits.append(act_lim)
        actuation_limits = np.array(actuation_limits)

        # Add floor
        dir_ground = np.array([0,0.0,1.0])
        p_ground = np.zeros((3,1))
        X_WG = RigidTransform(HalfSpace.MakePose(dir_ground,p_ground))
        #   Define friction
        friction = CoulombFriction(static_friction=self.settings.friction,dynamic_friction=self.settings.friction)
        #   Register ground contact
        plant.RegisterCollisionGeometry(plant.world_body(),X_WG,HalfSpace(),'ground',friction)
        plant.RegisterVisualGeometry(plant.world_body(),RigidTransform([0.,0.,-0.5]),Box(20,20,1),'ground',[0.9,0.9,0.9,1.])

        plant.Finalize()
        plant.SetDefaultPositions([1.,0.,0.,0., 0.,0.,0.325, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2])

        imu_body = plant.GetBodyByName("imu")

        contacts = [(plant.GetFrameByName("FL_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("FL_foot"))),
                    (plant.GetFrameByName("FR_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("FR_foot"))),
                    (plant.GetFrameByName("RL_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("RL_foot"))),
                    (plant.GetFrameByName("RR_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("RR_foot")))]

        WalkingRobot.__init__(self,builder,plant,scene_graph,imu_body,contacts,actuation_limits,dt=self.settings.controller_dt,is_sim=is_sim)

    def HardwareUpdate(self):
        pass


    