from pydrake.systems.framework import LeafSystem, Context, DiagramBuilder
from pydrake.common.value import AbstractValue
from pydrake.math import RotationMatrix
from pydrake.math import RotationMatrix, RollPitchYaw
from pydrake.common.eigen_geometry import Quaternion

import numpy as np

from ..robots.WalkingRobot import LLCActuationCommand, WalkingRobot
from ..observers.WalkingObserver import WalkingObserver
from ..targets.WalkingTarget import WalkingTarget, WalkingTargetValue

class WalkingController(LeafSystem):
    def __init__(self,n_act):
        LeafSystem.__init__(self)
        self.n_act = n_act
        self.actuation_data:LLCActuationCommand
        self.actuation_data = LLCActuationCommand(n_act)
        
        self.target_in = self.DeclareAbstractInputPort('target_in',AbstractValue.Make(WalkingTargetValue()))
        self.llc_out = self.DeclareAbstractOutputPort('llc_out',lambda: AbstractValue.Make(LLCActuationCommand(n_act)),self.CalcOutput)

    def CalcOutput(self,context: Context,output: AbstractValue):
        output.set_value(self.actuation_data)
        return
    
    def AddToBuilderAndConnect(self, builder: DiagramBuilder, robot: WalkingRobot, target: WalkingTarget = None, observer: WalkingObserver = None):
        builder.AddSystem(self)
        builder.Connect(self.llc_out,robot.llc_actuation_in)
        if observer is None:
            builder.Connect(robot.cheater_state_out,self.state_in)
        else:
            builder.Connect(observer.state_out,self.state_in)
        builder.Connect(observer.state_out,robot.state_in)
        if target is None:  
            target = WalkingTarget()
            builder.AddSystem(target)
        builder.Connect(target.target_value_out,self.target_in)
        
        return

def WorldToRobotCoordinates(qv_world: np.array, nq) -> np.array:
    qv_robot = qv_world.copy()
    qv_robot[4:6] = 0.
    rot = RollPitchYaw(Quaternion(qv_robot[:4]/np.linalg.norm(qv_robot[:4])))
    world_yaw = RotationMatrix(RollPitchYaw([0.,0.,rot.yaw_angle()]))
    qv_robot[4:6] = 0.
    yaw_inv = world_yaw.inverse()
    qv_robot[:4] = (yaw_inv.multiply(RotationMatrix(rot))).ToQuaternion().wxyz()
    qv_robot[nq:nq+3] = yaw_inv.multiply(qv_robot[nq:nq+3])
    qv_robot[nq+3:nq+6] = yaw_inv.multiply(qv_robot[nq+3:nq+6])

    return qv_robot