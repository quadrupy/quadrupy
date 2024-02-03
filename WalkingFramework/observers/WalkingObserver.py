from pydrake.systems.framework import LeafSystem, Context, DiagramBuilder
from pydrake.common.value import AbstractValue

import numpy as np

from ..robots.WalkingRobot import SensorData, WalkingRobot

class WalkingObserver(LeafSystem):
    def __init__(self,n_joints=12,n_contacts=4,config_dict=None):
        LeafSystem.__init__(self)
        self.n_state = 2*n_joints + 13 # [rotation (4), translation (3), joint_q (n_joints), ang_vel (3), trans_vel (3), joint_dq (n_joints)] 
        self.n_joints = n_joints
        self.n_contacts = n_contacts
        self.robot_state = np.zeros(self.n_state)
        
        self.sensing_in = self.DeclareAbstractInputPort('sensing_in',AbstractValue.Make(SensorData(n_joints,n_contacts)))
        self.state_out = self.DeclareVectorOutputPort('state_out',self.n_state,self.CalcOutput)

    def CalcOutput(self,context: Context,output: AbstractValue):
        output.set_value(self.robot_state)
        return
    
    def AddToBuilderAndConnect(self, builder: DiagramBuilder, robot: WalkingRobot):
        builder.AddSystem(self)
        builder.Connect(robot.sensing_out, self.sensing_in)
        
        return