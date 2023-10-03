from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, BasicVector, Context

from ControlFramework import Sensing, Observer, Controller, Target, ControlSystem

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse


def BuildDIPlant() -> "tuple[DiagramBuilder,MultibodyPlant,SceneGraph]":
    builder = DiagramBuilder()
    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 1e-3)
    parser = Parser(plant)
    parser.AddModels("DoubleIntegrator.urdf")
    plant.Finalize()
    return builder, plant, scene_graph


class DIPositionSensor(Sensing):
    def __init__(
        self,
        selection_matrix: np.ndarray = np.array([[1.0, 0.0]]),
        noise_covariance: np.array = None,
    ):
        self.selection_matrix = selection_matrix
        self.n_outputs = selection_matrix.shape[0]
        n_states = selection_matrix.shape[1]
        if noise_covariance is None:
            noise_covariance = np.zeros([self.n_outputs, self.n_outputs])
        self.noise_covariance = noise_covariance

        super().__init__(n_states, self.n_outputs)

    def CalcSensorData(self, context: Context, output: BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        noise = np.random.multivariate_normal(
            np.zeros(self.n_outputs), self.noise_covariance
        )

        output.SetFromVector(self.selection_matrix @ robot_state + noise)

class DIPositionAndAccelerationSensor(Sensing):
    def __init__(
        self,
        position_noise_covariance: np.array = None,
        acceleration_noise_covariance: np.array = None
    ):
        self.selection_matrix = np.array([[1.0, 0.0]])
        n_states = self.selection_matrix.shape[1]
        self.n_positions = 1
        self.n_accels = 1
        n_outputs = self.n_positions + self.n_accels

        if position_noise_covariance is None:
            position_noise_covariance = np.zeros([self.n_positions, self.n_positions])
        self.position_noise_covariance = position_noise_covariance

        if acceleration_noise_covariance is None:
            acceleration_noise_covariance = np.zeros([self.n_accels, self.n_accels])
        self.acceleration_noise_covariance = acceleration_noise_covariance

        super().__init__(n_states = n_states, n_accels = self.n_accels, n_outputs = n_outputs)

    def CalcSensorData(self, context: Context, output: BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        robot_acceleration = self.get_accelerations_input_port().Eval(context)
        position_noise = np.random.multivariate_normal(
            np.zeros(self.n_positions), self.position_noise_covariance
        )
        acceleration_noise = np.random.multivariate_normal(
            np.zeros(self.n_accels), self.acceleration_noise_covariance
        )

        output.SetFromVector(np.vstack([self.selection_matrix @ robot_state + position_noise, robot_acceleration + acceleration_noise]))

class DINaiveObserver(Observer):
    def __init__(self, n_sense=1, n_est_states=2, dt_update: float = 1e-3):
        super().__init__(n_sense, n_est_states)

        # Initialize our stored variables and position estimate
        self.prev_pos = 0.0
        self.prev_time = -1.0e-3
        self.est_state = np.zeros(2)

        # Periodically call UpdateEstimate to recompute the finite difference
        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update, 0.0, self.UpdateEstimate)

    def UpdateEstimate(self, context: Context, state):
        # Get the current simlator time
        curr_time = context.get_time()
        # Read in the sensor signal from the input port, and use this as our position estimate
        self.est_state[0] = self.get_sensor_input_port().Eval(context)
        # use the previous stored position estimate and time to do a finite difference for velocity
        self.est_state[1] = (self.est_state[0] - self.prev_pos) / (
            curr_time - self.prev_time
        )

        # Update the stored position and time
        self.prev_pos = self.est_state[0]
        self.prev_time = curr_time

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        output.SetFromVector(self.est_state)


class DIKalmanObserver(Observer):
    def __init__(self, n_sense = 1, n_est_states = 2, n_actuators = 1, dt_update: float = 1e-3):
        super().__init__(n_sense, n_est_states, n_actuators, dt_update)

        # Initialize the state estimate, state covariance, and process noise covariance
        self.x_k = np.zeros(n_est_states)
        self.P_k = np.eye(n_est_states)
        # self.Q_k = np.eye(n_est_states)  # Adjust as needed
        self.Q_k = np.eye(n_est_states)  # Adjust as needed

        # Measurement noise covariance (adjust as needed)
        self.R_k = np.array([[0.0001]])

        # State transition matrix (A) for a continuous-time double integrator
        self.F = np.array([[1.0, dt_update], [0.0, 1.0]])
        self.B = np.array([0.0, dt_update])

        # Measurement matrix (C)
        self.H = np.array([[1.0, 0.0]])

        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update, 0.0, self.UpdateEstimate)

    def UpdateEstimate(self, context: Context, state):
        # Get current sensor reading and previous actuation value
        z_k = self.get_sensor_input_port().Eval(context)
        u_k_minus_1 = self.get_actuation_input_port().Eval(context)

        # Prediction step: Propagate the state estimate and covariance forward
        x_k_prior = self.F @ self.x_k + self.B * u_k_minus_1
        P_k_prior = self.F @ self.P_k @ self.F.T + self.Q_k

        # Correction step: Calculate Kalman gain and update the estimate and covariance
        y_k = z_k - self.H @ x_k_prior
        S_k = self.H @ P_k_prior @ self.H.T + self.R_k
        K_k = P_k_prior @ self.H.T @ np.linalg.inv(S_k)

        self.x_k = x_k_prior + K_k @ y_k
        self.P_k = P_k_prior - K_k @ self.H @ P_k_prior

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        # Reshape est_state to match the desired shape (2,)
        output.SetFromVector(np.squeeze(self.x_k))


class DIKalmanObserverWithAcceleration(Observer):
    def __init__(self, n_sense = 2, n_est_states = 2, n_actuators = 0, dt_update: float = 1e-3):
        super().__init__(n_sense, n_est_states, n_actuators, dt_update)

        # Initialize the state estimate, state covariance, and process noise covariance
        self.x_k = np.zeros(n_est_states)
        self.P_k = np.eye(n_est_states)
        # self.Q_k = np.eye(n_est_states)  # Adjust as needed
        self.Q_k = np.eye(n_est_states)  # Adjust as needed

        # Measurement noise covariance (adjust as needed)
        self.R_k = np.array([[0.0001]])

        # State transition matrix (A) for a continuous-time double integrator
        self.F = np.array([[1.0, dt_update], [0.0, 1.0]])

        # Measurement matrix (C)
        self.H = np.array([[1.0, 0.0]])

        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update, 0.0, self.UpdateEstimate)

    def UpdateEstimate(self, context: Context, state):
        # Get current sensor reading and previous actuation value
        sensing_k = self.get_sensor_input_port().Eval(context)
        z_k = sensing_k[0]
        acceleration_k = sensing_k[1]

        # Prediction step: Propagate the state estimate and covariance forward
        x_k_prior = self.F @ self.x_k
        P_k_prior = self.F @ self.P_k @ self.F.T + self.Q_k

        # Correction step: Calculate Kalman gain and update the estimate and covariance
        y_k = z_k[0] - self.H @ x_k_prior
        S_k = self.H @ P_k_prior @ self.H.T + self.R_k
        K_k = P_k_prior @ self.H.T @ np.linalg.inv(S_k)

        self.x_k = x_k_prior + K_k @ y_k
        self.P_k = P_k_prior - K_k @ self.H @ P_k_prior

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        # Reshape est_state to match the desired shape (2,)
        output.SetFromVector(np.squeeze(self.x_k))

class DIController(Controller):
    def __init__(
        self,
        K: np.ndarray = np.array([[100.0, 50.0]]),
        max_force: np.ndarray = np.array([100.0]),
    ):
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


class DITarget(Target):
    def __init__(self, omega: float = 1.0):
        n_targets = 2
        self.omega = omega
        super().__init__(n_targets)

    def CalcTargetOutputs(self, context: Context, output: BasicVector):
        t = context.get_time()
        des_pos = np.cos(self.omega * t) - 1
        des_vel = -self.omega * np.sin(self.omega * t)
        output.SetAtIndex(0, des_pos)
        output.SetAtIndex(1, des_vel)


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="Simulate a double integrator control system"
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
    builder, plant, scene_graph = BuildDIPlant()
    # Build the control system
    system = ControlSystem(
        builder,
        plant,
        scene_graph,
        DIPositionAndAccelerationSensor(position_noise_covariance=[[args.pos_noise_std**2]],acceleration_noise_covariance=[[args.acc_noise_std**2]]),
        DIKalmanObserverWithAcceleration(),
        DIController(),
        DITarget(),
    )
    # Simulate the control system
    log_data = system.Simulate(np.array([-1., -1.]),20, wait=False)

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
    plt.ylabel("Position (m)")
    plt.legend()

    plt.figure()
    plt.plot(time, true_state[:, 1], label="Actual", zorder=2)
    plt.plot(time, est_state[:, 1], "--", label="Estimated", zorder=1)
    plt.plot(time, ref_state[:, 1], ":", label="Desired", zorder=3)
    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.legend()

    plt.show()
