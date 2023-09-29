from __future__ import annotations

from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, Role, StartMeshcat, SceneGraph
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, Diagram, LeafSystem, InputPort, OutputPort, BasicVector, Context
from pydrake.systems.analysis import Simulator

import numpy as np

class Sensing(LeafSystem):
    def __init__(self,n_states,n_outputs):
        super().__init__()

        self.DeclareVectorInputPort('state_input', n_states)
        self.DeclareVectorOutputPort('sensor_output', n_outputs, self.CalcSensorData)

    def get_state_input_port(self) -> InputPort:
        return self.GetInputPort('state_input')

    def get_sensor_output_port(self) -> OutputPort:
        return self.GetOutputPort('sensor_output')

    def CalcSensorData(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_sensor_output_port().Eval(context)

class Observer(LeafSystem):
    def __init__(self,n_sensors,n_est_states):
        super().__init__()

        self.DeclareVectorInputPort('sensor_input', n_sensors)
        self.DeclareVectorOutputPort('est_state_output', n_est_states, self.CalcObserverOutput)
    
    def get_sensor_input_port(self) -> InputPort:
        return self.GetInputPort('sensor_input')

    def get_estimated_state_output_port(self) -> OutputPort:
        return self.GetOutputPort('est_state_output')

    def CalcObserverOutput(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_estimated_state_output_port().Eval(context)
    
class Controller(LeafSystem):
    def __init__(self, n_states, n_targets, n_outputs):
        super().__init__()

        self.DeclareVectorInputPort('state_input', n_states)
        self.DeclareVectorInputPort('target_input', n_targets)
        self.DeclareVectorOutputPort('actuation_output', n_outputs, self.CalcControllerOutput)

    def get_state_input_port(self) -> InputPort:
        return self.GetInputPort('state_input')

    def get_target_input_port(self) -> InputPort:
        return self.GetInputPort('target_input')

    def get_actuation_output_port(self) -> OutputPort:
        return self.GetOutputPort('actuation_output')
    
    def CalcControllerOutput(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_actuation_output_port().Eval(context)
    
class Target(LeafSystem):
    def __init__(self, n_targets):
        super().__init__()

        self.DeclareVectorOutputPort('target_output', n_targets, self.CalcTargetOutputs)
        
    def get_target_output_port(self) -> OutputPort:
        return self.GetOutputPort('target_output')

    def CalcTargetOutputs(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_target_output_port().Eval(context)
    
class ControlSystem():
    def __init__(self,builder: DiagramBuilder, plant: MultibodyPlant, scene_graph: SceneGraph, sensing: Sensing,observer: Observer,controller: Controller,target: Target):
        self.plant:MultibodyPlant = plant
        self.scene_graph:SceneGraph = scene_graph
        self.sensing:Sensing = sensing
        self.observer:Observer  = observer
        self.controller:Controller = controller
        self.target:Target = target
        self.log_buffer = {'time':[],'plant_data':[],'sensing_data':[],'observer_data':[],'controller_data':[],'target_data':[]}
        
        # Add the componenets to the builder and connect everything
        builder.AddSystem(sensing)
        builder.AddSystem(observer)
        builder.AddSystem(controller)
        builder.AddSystem(target)
        builder.Connect(plant.get_state_output_port(), sensing.get_state_input_port())
        builder.Connect(sensing.get_sensor_output_port(),observer.get_sensor_input_port())
        builder.Connect(observer.get_estimated_state_output_port(),controller.get_state_input_port())
        builder.Connect(target.get_target_output_port(),controller.get_target_input_port())
        builder.Connect(controller.get_actuation_output_port(),plant.get_actuation_input_port())

        meshcat = StartMeshcat()
        vis_params = MeshcatVisualizerParams(role=Role.kPerception, prefix="visual",publish_period=1/64.)
        self.visualizer:MeshcatVisualizer = MeshcatVisualizer.AddToBuilder(builder, self.scene_graph, meshcat, vis_params)

        self.diagram:Diagram = builder.Build()
        self.simulator:Simulator = Simulator(self.diagram)
        self.simulator.set_target_realtime_rate(1.e3)
        self.simulator.set_monitor(self.Logger)

    def Simulate(self,init_state:np.array,sim_time:float,wait=True)->dict:
        self.simulator.Initialize()
        self.log_buffer = {'time':[],'plant_data':[],'sensing_data':[],'observer_data':[],'controller_data':[],'target_data':[]}

        sim_context = self.plant.GetMyContextFromRoot(self.simulator.get_context())
        self.plant.SetPositionsAndVelocities(sim_context,init_state)

        self.visualizer.StartRecording()

        self.simulator.AdvanceTo(sim_time)

        self.visualizer.StopRecording()
        self.visualizer.PublishRecording()

        if wait:
            input('Hit enter when done viewing')

        return self.log_buffer

    def Logger(self,context:Context):
        self.log_buffer['time'].append(context.get_time())
        self.log_buffer['plant_data'].append(self.plant.GetPositionsAndVelocities(self.plant.GetMyContextFromRoot(context)))
        self.log_buffer['sensing_data'].append(self.sensing.LogData(self.sensing.GetMyContextFromRoot(context)))
        self.log_buffer['observer_data'].append(self.observer.LogData(self.observer.GetMyContextFromRoot(context)))
        self.log_buffer['controller_data'].append(self.controller.LogData(self.controller.GetMyContextFromRoot(context)))
        self.log_buffer['target_data'].append(self.target.LogData(self.target.GetMyContextFromRoot(context)))
        