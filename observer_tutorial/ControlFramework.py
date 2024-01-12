from __future__ import annotations

from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, Role, StartMeshcat, SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, Diagram, LeafSystem, InputPort, OutputPort, BasicVector, Context
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import ZeroOrderHold

import numpy as np

class Sensing(LeafSystem):
    # The Sensing class takes the state of the system and computes a sensor output signal
    # For example, an encoder will return a (potentially noisy) value corresponding to a joint angle
    def __init__(self,n_states,n_outputs,n_accels = 0):
        super().__init__()
        self.n_accels = n_accels

        self.DeclareVectorInputPort('state_input', n_states)
        if n_accels > 0:
            self.DeclareVectorInputPort('acceleration_input', n_accels)
        self.DeclareVectorOutputPort('sensor_output', n_outputs, self.CalcSensorData)

    def get_state_input_port(self) -> InputPort:
        return self.GetInputPort('state_input')

    def get_accelerations_input_port(self) -> InputPort:
        if self.n_accels == 0:
            return None
        else:
            return self.GetInputPort('acceleration_input')

    def get_sensor_output_port(self) -> OutputPort:
        return self.GetOutputPort('sensor_output')

    def CalcSensorData(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_sensor_output_port().Eval(context)

class FullStateSensing(Sensing):
    def __init__(self,n_states):
        n_outputs = n_states
        n_accels = 0
        super().__init__(n_states,n_outputs,n_accels)

    def CalcSensorData(self, context: Context, output: BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        output.SetFromVector(robot_state)
        return

class Observer(LeafSystem):
    # The Observer class takes the raw sensor data and tries to estimate the system state
    # For example this might take a finite difference on a position signal to estimate a velocity
    def __init__(self,n_sensors,n_est_states,n_actuators = 0,dt_update = None):
        super().__init__()
        self.n_actuators = n_actuators
        self.dt_update = dt_update

        self.DeclareVectorInputPort('sensor_input', n_sensors)
        if self.n_actuators > 0:
            self.DeclareVectorInputPort('actuation_input', n_actuators)
        self.DeclareVectorOutputPort('est_state_output', n_est_states, self.CalcObserverOutput)
    
    def get_sensor_input_port(self) -> InputPort:
        return self.GetInputPort('sensor_input')

    def get_actuation_input_port(self) -> InputPort:
        if self.n_actuators == 0:
            return None
        else:
            return self.GetInputPort('actuation_input')
    
    def get_estimated_state_output_port(self) -> OutputPort:
        return self.GetOutputPort('est_state_output')

    def CalcObserverOutput(self,context:Context,output:BasicVector):
        output.SetZero()
        return
    
    def LogData(self,context:Context):
        return self.get_estimated_state_output_port().Eval(context)
    
class FullStateObserver(Observer):
    def __init__(self,n_states):
        n_sensors = n_states
        n_est_states = n_states
        n_actuators = 0
        dt_update = None
        super().__init__(n_sensors,n_est_states,n_actuators,dt_update)

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        sensor_state = self.get_sensor_input_port().Eval(context)
        output.SetFromVector(sensor_state)
        return

class Controller(LeafSystem):
    # The Controller class takes the estimated state from the observer and a target value, and computes
    # an actuation input to the system. For example a PD controller scales the tracking error by a constant
    # to find an output torque.
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
    # The Target class generates the reference signal to be tracked. This might be a fixed value, or might vary with time.
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
    # The ControlSystem puts all the puzzle pieces together in a Diagram and connexcts their inputs and outputs. 
    # it also provides a simulator, visualizer, and logger that show the response of the system.
    # To access the visualization output, open localhost:7000/ in your browser
    def __init__(self,
                 builder: DiagramBuilder, 
                 plant: MultibodyPlant, 
                 scene_graph: SceneGraph, 
                 sensing: Sensing,
                 observer: Observer,
                 controller: Controller,
                 target: Target):
        self.plant:MultibodyPlant = plant
        self.scene_graph:SceneGraph = scene_graph
        if sensing is None:
            # Use full state feedback if no sensing is provided
            sensing = FullStateSensing(plant.num_multibody_states())
            observer = FullStateObserver(plant.num_multibody_states())
        self.sensing:Sensing = sensing
        self.observer:Observer  = observer
        self.controller:Controller = controller
        self.target:Target = target
        self.log_buffer = {'time':[],'plant_data':[],'sensing_data':[],'observer_data':[],'controller_data':[],'target_data':[]}

        # Add the components to the builder
        builder.AddSystem(sensing)
        builder.AddSystem(observer)
        builder.AddSystem(controller)
        builder.AddSystem(target)

        # Connect the input ports and output ports
        builder.Connect(plant.get_state_output_port(), sensing.get_state_input_port())
        if sensing.get_accelerations_input_port() is not None:
            acc_zoh_dt = self.plant.time_step()
            self.acceleration_hold = ZeroOrderHold(acc_zoh_dt, plant.get_generalized_acceleration_output_port().size(), 0.0)
            builder.AddSystem(self.acceleration_hold)
            builder.Connect(plant.get_generalized_acceleration_output_port(), self.acceleration_hold.get_input_port())
            builder.Connect(self.acceleration_hold.get_output_port(), sensing.get_accelerations_input_port())

        builder.Connect(sensing.get_sensor_output_port(), observer.get_sensor_input_port())
        if observer.get_actuation_input_port() is not None:
            # Observer depends on the previous sensor input. Add a zero-order-hold and connect
            if self.observer.dt_update is None:
                act_zoh_dt = self.plant.time_step()
            else:
                act_zoh_dt = self.observer.dt_update
            self.actuation_hold = ZeroOrderHold(act_zoh_dt, controller.get_actuation_output_port().size(), 0.0)
            builder.AddSystem(self.actuation_hold)
            builder.Connect(controller.get_actuation_output_port(), self.actuation_hold.get_input_port())
            builder.Connect(self.actuation_hold.get_output_port(), observer.get_actuation_input_port())
            
        builder.Connect(observer.get_estimated_state_output_port(), controller.get_state_input_port())
        builder.Connect(target.get_target_output_port(), controller.get_target_input_port())
        builder.Connect(controller.get_actuation_output_port(), plant.get_actuation_input_port())

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
        