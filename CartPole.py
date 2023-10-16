from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, BasicVector, Context
from pydrake.systems.primitives import Linearize
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator

from ControlFramework import Sensing, Observer, Controller, Target, ControlSystem

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse

def BuildCPPlant() -> "tuple[DiagramBuilder,MultibodyPlant,SceneGraph]":
    # See http://underactuated.mit.edu/acrobot.html#cart_pole for a description of the cartpole system
    builder = DiagramBuilder()
    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    parser = Parser(plant)
    parser.AddModels("CartPole.urdf")
    plant.Finalize()
    return builder, plant, scene_graph

class CPPositionAndAccelerationSensor(Sensing):
    def __init__(
        self,
        position_noise_covariance: np.array = None,
        acceleration_noise_covariance: np.array = None
    ):
        self.position_selection_matrix = np.array([[0.0, 1.0, 0.0, 0.0]])
        self.acceleration_selection_matrix = np.array([[1.0, 0.0]])
        n_states = self.position_selection_matrix.shape[1]
        n_accels = self.acceleration_selection_matrix.shape[1]
        self.n_positions = self.position_selection_matrix.shape[0]
        self.n_accel_meas = self.acceleration_selection_matrix.shape[0]
        n_outputs = self.n_positions + self.n_accel_meas

        if position_noise_covariance is None:
            position_noise_covariance = np.zeros([self.n_positions, self.n_positions])
        self.position_noise_covariance = position_noise_covariance

        if acceleration_noise_covariance is None:
            acceleration_noise_covariance = np.zeros([self.n_accels, self.n_accels])
        self.acceleration_noise_covariance = acceleration_noise_covariance

        super().__init__(n_states = n_states, n_accels = n_accels, n_outputs = n_outputs)

    def CalcSensorData(self, context: Context, output: BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        robot_acceleration = self.get_accelerations_input_port().Eval(context)
        position_noise = np.random.multivariate_normal(
            np.zeros(self.n_positions), self.position_noise_covariance
        )
        acceleration_noise = np.random.multivariate_normal(
            np.zeros(self.n_accel_meas), self.acceleration_noise_covariance
        )

        output.SetFromVector(np.vstack([self.position_selection_matrix @ robot_state + position_noise, self.acceleration_selection_matrix @ robot_acceleration + acceleration_noise]))

class CPKalmanObserver(Observer):
    def __init__(self, n_sense = 2, n_est_states = 4, n_actuators = 0, dt_update: float = 1e-3):
        super().__init__(n_sense, n_est_states, n_actuators, dt_update)
        self.x_k = np.zeros(n_est_states)

        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update, 0.0, self.UpdateEstimate)

    def UpdateEstimate(self, context: Context, state):
        # Get current sensor reading and previous actuation value
        z_k = self.get_sensor_input_port().Eval(context)

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        # Reshape est_state to match the desired shape (2,)
        output.SetFromVector(np.squeeze(self.x_k))

class CPLQRController(Controller):
    # See http://underactuated.mit.edu/lqr.html for an overview on LQR
    def __init__(
        self,
        plant: MultibodyPlant,
        Q: np.ndarray = np.diag(np.array([10.0, 1.0, 1.0, 1.0])),
        R: np.ndarray = np.array([[1.0]]),
        max_force: np.ndarray = np.array([100.0]),
    ):
        linearization_context = plant.CreateDefaultContext()
        plant.SetPositionsAndVelocities(linearization_context, np.array([0., np.pi, 0., 0.]))
        plant.get_actuation_input_port().FixValue(linearization_context,0.)
        linearization = Linearize(plant,
                                  linearization_context,
                                  plant.get_actuation_input_port().get_index(),
                                  plant.get_state_output_port().get_index())
        
        K,_ = DiscreteTimeLinearQuadraticRegulator(linearization.A(),
                                                   linearization.B(),
                                                   Q,
                                                   R)

        n_states = K.shape[1]
        n_outputs = K.shape[0]
        self.K = K
        self.max_force = max_force
        super().__init__(n_states, n_states, n_outputs)

    def CalcControllerOutput(self, context: Context, output: BasicVector):
        est_robot_state = self.get_state_input_port().Eval(context)
        target_state = self.get_target_input_port().Eval(context)
        act_out = np.clip(
            self.K @ (target_state - est_robot_state), -self.max_force, self.max_force
        )
        output.SetFromVector(act_out)


class CPTarget(Target):
    def __init__(self):
        n_targets = 4
        self.target_value = np.array([0., np.pi, 0., 0.])
        super().__init__(n_targets)

    def CalcTargetOutputs(self, context: Context, output: BasicVector):
        output.SetFromVector(self.target_value)

if __name__ == "__main__":
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
        "-pos_sensor_noise",
        dest="pos_noise_std",
        default=0.0,
        type=float,
        help="Add position sensor noise with given std (0.01 is a good start!)",
    )
    parser.add_argument(
        "-acc_sensor_noise",
        dest="acc_noise_std",
        default=0.0,
        type=float,
        help="Add acceleration sensor noise with given std (0.01 is a good start!)",
    )
    args = parser.parse_args()

    # Build the plant
    builder, plant, scene_graph = BuildCPPlant()
    # Build the control system
    if args.cheater_observer:
        sensing = None
    else:
        sensing = CPPositionAndAccelerationSensor(position_noise_covariance=[[args.pos_noise_std**2]],
                                                  acceleration_noise_covariance=[[args.acc_noise_std**2]])

    system = ControlSystem(
        builder,
        plant,
        scene_graph,
        sensing,
        CPKalmanObserver(),
        CPLQRController(plant),
        CPTarget(),
    )
    # Simulate the control system
    log_data = system.Simulate(np.array([-1., np.pi-0.5, 0., 0.]),20, wait=False)

    # Parse the logged data and plot the tracking performance
    time = np.array(log_data["time"])
    true_state = np.array(log_data["plant_data"])
    ref_state = np.array(log_data["target_data"])
    est_state = np.array(log_data["observer_data"])

    plt.figure()
    plt.plot(time, true_state[:, 0], label="Actual", zorder=2)
    plt.plot(time, est_state[:, 0], "--", label="Estimated", zorder=1)
    plt.plot(time, ref_state[:, 0], ":", label="Desired", zorder=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Cart Position (m)")
    plt.legend()

    plt.figure()
    plt.plot(time, true_state[:, 1], label="Actual", zorder=2)
    plt.plot(time, est_state[:, 1], "--", label="Estimated", zorder=1)
    plt.plot(time, ref_state[:, 1], ":", label="Desired", zorder=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Pole Position (rad)")
    plt.legend()

    plt.figure()
    plt.plot(time, true_state[:, 2], label="Actual", zorder=2)
    plt.plot(time, est_state[:, 2], "--", label="Estimated", zorder=1)
    plt.plot(time, ref_state[:, 2], ":", label="Desired", zorder=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Cart Velocity (m/s)")
    plt.legend()

    plt.figure()
    plt.plot(time, true_state[:, 3], label="Actual", zorder=2)
    plt.plot(time, est_state[:, 3], "--", label="Estimated", zorder=1)
    plt.plot(time, ref_state[:, 3], ":", label="Desired", zorder=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Pole Velocity (rad/s)")
    plt.legend()


    plt.show()
