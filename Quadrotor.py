from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, BasicVector, Context
from pydrake.systems.primitives import Linearize, LinearSystem, Demultiplexer, Multiplexer
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator
from pydrake.examples import QuadrotorPlant, QuadrotorGeometry
from pydrake.geometry import MeshcatVisualizer, MeshcatVisualizerParams, Role, StartMeshcat, SceneGraph
from pydrake.multibody.plant import MultibodyPlant, Propeller, ExternallyAppliedSpatialForceMultiplexer
from pydrake.systems.framework import DiagramBuilder, Diagram, LeafSystem, InputPort, OutputPort, BasicVector, Context, System, OutputPortIndex, InputPortIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.primitives import ZeroOrderHold

from ControlFramework import Sensing, Observer, Controller, Target, ControlSystem

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse
import types

def BuildQuadrotorPlant(dt) -> "tuple[DiagramBuilder,QuadrotorPlant,SceneGraph]":
    builder = DiagramBuilder()
    scene_graph = SceneGraph()
    plant = QuadrotorSystem(dt)
    builder.AddSystem(plant.diagram)
    builder.AddSystem(scene_graph)

    vis_state_converter = RobotStateToVisStateConverter()
    builder.AddSystem(vis_state_converter)
    builder.Connect(plant.get_state_output_port(), vis_state_converter.get_input_port())
    QuadrotorGeometry.AddToBuilder(builder, vis_state_converter.get_output_port(), scene_graph)

    return builder, plant, scene_graph

class QuadrotorSystem():
    def __init__(self, dt) -> None:
        builder = DiagramBuilder()
        plant: MultibodyPlant
        plant, _ = AddMultibodyPlantSceneGraph(builder, dt)
        parser = Parser(plant)
        parser.AddModels("quadrotor.urdf")
        plant.Finalize()
        self.body_plant = plant

        # Define the propellers
        prop_locs = np.array([[0.1,0.15,0.],[0.1,-0.15,0.],[-0.1,0.15,0.],[-0.1,-0.15,0.]])
        prop_moment_ratio = np.array([0.1,-0.1,-0.1,0.1])
        n_props = prop_locs.shape[0]
        self.prop_input_demux = Demultiplexer(n_props,1)
        builder.AddSystem(self.prop_input_demux)
        self.prop_output_mux = ExternallyAppliedSpatialForceMultiplexer(n_props)
        builder.AddSystem(self.prop_output_mux)
        for i in range(n_props):
            prop = Propeller(plant.GetBodyByName("base_link").index(), X_BP=RigidTransform(prop_locs[i,:]), moment_ratio=prop_moment_ratio[i])
            builder.AddSystem(prop)
            builder.Connect(prop.GetOutputPort("spatial_forces"), self.prop_output_mux.get_input_port(i))
            builder.Connect(plant.get_body_poses_output_port(), prop.GetInputPort("body_poses"))
            builder.Connect(self.prop_input_demux.get_output_port(i), prop.get_command_input_port())
        builder.Connect(self.prop_output_mux.get_output_port(), plant.get_applied_spatial_force_input_port())

        # Declare input and output ports
        self.actuation_input: InputPortIndex = builder.ExportInput(self.prop_input_demux.get_input_port(), "prop_input")
        self.state_output: OutputPortIndex = builder.ExportOutput(plant.get_state_output_port(), "state_output")
        self.acceleration_output: OutputPortIndex = builder.ExportOutput(plant.get_generalized_acceleration_output_port(), "acceleration_output")

        # Build the diagram
        self.diagram: Diagram
        self.diagram = builder.Build()

    def get_state_output_port(self):
        return self.diagram.get_output_port(self.state_output)
    
    def get_actuation_input_port(self):
        return self.diagram.get_input_port(self.actuation_input)
    
    def get_generalized_acceleration_output_port(self):
        return self.diagram.get_output_port(self.acceleration_output)
    
    def GetMyContextFromRoot(self, context: Context):
        return self.body_plant.GetMyContextFromRoot(context)

    def SetPositionsAndVelocities(self, context: Context, x: np.ndarray):
        self.body_plant.SetPositionsAndVelocities(context, x)

    def GetPositionsAndVelocities(self, context: Context):
        return self.body_plant.GetPositionsAndVelocities(context)

    def num_multibody_states(self):
        return self.body_plant.num_multibody_states()
    
    def time_step(self):
        return self.body_plant.time_step()
        
class RobotStateToVisStateConverter(LeafSystem):
    def __init__(self):
        super().__init__()
        self.DeclareVectorInputPort("quat_state", BasicVector(13))
        self.DeclareVectorOutputPort("rpy_state", BasicVector(12), self.CalcOutput)
    
    def CalcOutput(self, context: Context, output: BasicVector):
        robot_state = self.get_input_port().Eval(context) # [qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz]
        quat = robot_state[:4]
        rpy = RollPitchYaw(Quaternion(quat/np.linalg.norm(quat)))

        vis_state = np.hstack([robot_state[4:7],rpy.vector(),robot_state[10:13],robot_state[7:10]]) # [x,y,z,r,p,y,vx,vy,vz,wx,wy,wz]
        output.SetFromVector(vis_state)

def LinearizeQuadrotorPlant(plant: QuadrotorSystem, x_0: np.array, u_0: np.array) -> LinearSystem:
    linearization_context = plant.diagram.CreateDefaultContext()
    plant.SetPositionsAndVelocities(plant.GetMyContextFromRoot(linearization_context), x_0)
    plant.get_actuation_input_port().FixValue(linearization_context,u_0)
    linearization = Linearize(plant.diagram,
                              linearization_context,
                              plant.get_actuation_input_port().get_index(),
                              plant.get_state_output_port().get_index())
    return linearization

class QuadrotorSensing(Sensing):
    def __init__(self, accel_noise_std = 0.01, ang_vel_noise_std = 0.01):
        n_states = 13
        n_accel_in = 6 # 6 spatial accelerations coming from the robot
        n_accel_meas = 3 # We only measure acceleration in x, y, and z
        n_gyro = 3 # We also measure wx, wy, wz
        n_outputs = n_accel_meas + n_gyro
        self.accel_noise_std = accel_noise_std
        self.ang_vel_noise_std = ang_vel_noise_std

        super().__init__(n_states = n_states,n_outputs = n_outputs,n_accels = n_accel_in)

    def CalcSensorData(self, context: Context, output: BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        robot_rot = RotationMatrix(Quaternion(robot_state[:4]/np.linalg.norm(robot_state[:4])))
        angular_vel_world_frame = robot_state[7:10] # [wx,wy,wz]
        linear_accel_world_frame = self.get_accelerations_input_port().Eval(context)[3:6] + np.array([0.,0.,9.81]) # [ax,ay,az]

        linear_accel = robot_rot.inverse().multiply(linear_accel_world_frame)
        angular_vel = robot_rot.inverse().multiply(angular_vel_world_frame)

        accel_noise = np.random.normal(0.,self.accel_noise_std,3)
        angular_vel_noise = np.random.normal(0.,self.ang_vel_noise_std,3)

        output.SetFromVector(np.hstack([linear_accel + accel_noise,angular_vel + angular_vel_noise]))

class QuadrotorKalmanObserver(Observer):
    def __init__(self,dt_update):
        n_sensors = 6
        n_est_states = 13
        n_actuators = 0

        super().__init__(n_sensors,n_est_states,n_actuators,dt_update)

    def CalcObserverState(self, context: Context, output: BasicVector):
        # Get current sensor readings
        sensor_data = self.get_sensor_input_port().Eval(context)
        linear_acceleration_body_frame = sensor_data[:3]
        angular_velocity_body_frame = sensor_data[3:]

        # Compute the current state estimate

class QuadrotorLQRController(Controller):
    # See http://underactuated.mit.edu/lqr.html for an overview on LQR
    def __init__(
        self,
        discrete_time_linearized_plant: LinearSystem,
        u_0: np.ndarray = np.zeros(4),
        Q: np.ndarray = np.diag(np.ones(12)),
        R: np.ndarray = np.diag(np.ones(4)),
        max_force: np.ndarray = np.array([10.0]),
    ):
        self.u_0 = u_0

        # To linearize the quaternion state, use qx,qy,qz as the state variables.
        K,_ = DiscreteTimeLinearQuadraticRegulator( discrete_time_linearized_plant.A()[1:,1:],
                                                    discrete_time_linearized_plant.B()[1:,:],
                                                    Q,
                                                    R)

        n_states = K.shape[1] + 1
        n_outputs = K.shape[0]
        self.K = K
        self.max_force = max_force
        super().__init__(n_states, n_states, n_outputs)

    def CalcControllerOutput(self, context: Context, output: BasicVector):
        est_robot_state = self.get_state_input_port().Eval(context)
        target_state = self.get_target_input_port().Eval(context)

        est_robot_rot = RotationMatrix(Quaternion(est_robot_state[:4]/np.linalg.norm(est_robot_state[:4])))
        target_rot = RotationMatrix(Quaternion(target_state[:4]/np.linalg.norm(target_state[:4])))

        # To get rotation error in body frame we use an axis_angle representation of the difference between the two rotations
        rotation_error_body_frame = est_robot_rot.InvertAndCompose(target_rot).ToAngleAxis()
        rotation_error_body_frame = rotation_error_body_frame.angle() * rotation_error_body_frame.axis()

        position_error_body_frame = est_robot_rot.inverse().multiply(target_state[4:7] - est_robot_state[4:7])
        angular_velocity_error_body_frame = est_robot_rot.inverse().multiply(target_state[7:10] - est_robot_state[7:10])
        linear_velocity_error_body_frame = est_robot_rot.inverse().multiply(target_state[10:13] - est_robot_state[10:13])

        state_error = np.hstack([rotation_error_body_frame, position_error_body_frame, angular_velocity_error_body_frame, linear_velocity_error_body_frame])
        
        act_out = np.clip(
            self.K @ (state_error) + self.u_0, -self.max_force, self.max_force
        )
        output.SetFromVector(act_out)

class QuadrotorTarget(Target):
    def __init__(self):
        n_targets = 13
        self.target_value = np.zeros(n_targets)
        self.target_value[0] = 1.   
        super().__init__(n_targets)

    def CalcTargetOutputs(self, context: Context, output: BasicVector):
        output.SetFromVector(self.target_value)

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Simulate a cart-pole control system"
    )
    parser.add_argument(
        "-cheater_observer",
        dest="cheater_observer",
        default=False,
        type=bool,
        help="Use a cheater observer (true) or a Kalman filter (false)",
    )
    parser.add_argument(
        "-ang_vel_sensor_noise",
        dest="ang_vel_noise_std",
        default=0.01,
        type=float,
        help="Add angular velocity sensor noise with given std (0.01 is a good start!)",
    )
    parser.add_argument(
        "-acc_sensor_noise",
        dest="acc_noise_std",
        default=0.01,
        type=float,
        help="Add acceleration sensor noise with given std (0.01 is a good start!)",
    )
    args = parser.parse_args()

    plant: QuadrotorSystem
    dt = 1e-3
    builder, plant, scene_graph = BuildQuadrotorPlant(dt)
    q_init = RollPitchYaw(np.pi,0.,0.).ToQuaternion().wxyz()
    x_init = np.hstack([q_init,[1.,1.,-1.,0.,0.,0.,0.,0.,0.]])

    # Compute linearization for LQR
    u_0 = np.ones(4)*9.81*plant.body_plant.CalcTotalMass(plant.body_plant.CreateDefaultContext())/4
    x_0 = np.zeros(13)
    x_0[0] = 1.
    linearized_plant = LinearizeQuadrotorPlant(plant, x_0, u_0)


    controller = QuadrotorLQRController(linearized_plant,u_0=u_0)
    if args.cheater_observer:
        observer = None
    else:
        sensing = QuadrotorSensing(accel_noise_std=args.acc_noise_std, ang_vel_noise_std=args.ang_vel_noise_std)
    observer = QuadrotorKalmanObserver(dt)
    target = QuadrotorTarget()

    system = ControlSystem(builder=builder,plant=plant,scene_graph=scene_graph,controller=controller,target=target,sensing=None,observer=observer)

    log_data = system.Simulate(x_init,10.,wait=False)

    # Parse the logged data and plot the tracking performance
    time = np.array(log_data["time"])
    true_state = np.array(log_data["plant_data"])
    ref_state = np.array(log_data["target_data"])
    est_state = np.array(log_data["observer_data"])

    robot_rpy = np.array([RollPitchYaw(Quaternion(true_state[i,0:4]/np.linalg.norm(true_state[i,0:4]))).vector() for i in range(true_state.shape[0])])
    plt.plot(time,robot_rpy[:,0],label="Roll")
    plt.plot(time,robot_rpy[:,1],label="Pitch")
    plt.plot(time,robot_rpy[:,2],label="Yaw")
    plt.title("Robot Orientation (World Frame)")
    plt.legend()

    robot_pos = true_state[:,4:7]
    plt.figure()
    plt.plot(time,robot_pos[:,0],label="x")
    plt.plot(time,robot_pos[:,1],label="y")
    plt.plot(time,robot_pos[:,2],label="z")
    plt.title("Robot Position (World Frame)")
    plt.legend()

    plt.show()