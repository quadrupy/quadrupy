from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, BasicVector, Context
from pydrake.systems.primitives import Linearize, LinearSystem
from pydrake.systems.controllers import DiscreteTimeLinearQuadraticRegulator

from ControlFramework import Sensing, Observer, Controller, Target, ControlSystem

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse

def BuildCPPlant(dt) -> "tuple[DiagramBuilder,MultibodyPlant,SceneGraph]":
    # See http://underactuated.mit.edu/acrobot.html#cart_pole for a description of the cartpole system
    builder = DiagramBuilder()
    plant: MultibodyPlant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, dt)
    parser = Parser(plant)
    parser.AddModels("CartPole.urdf")
    plant.Finalize()
    return builder, plant, scene_graph

def LinearizeCPPlant(plant: MultibodyPlant, x_0: np.array) -> LinearSystem:
    linearization_context = plant.CreateDefaultContext()
    plant.SetPositionsAndVelocities(linearization_context, np.array(x_0))
    plant.get_actuation_input_port().FixValue(linearization_context,0.)
    linearization = Linearize(plant,
                              linearization_context,
                              plant.get_actuation_input_port().get_index(),
                              plant.get_state_output_port().get_index())
    return linearization

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
    def __init__(self, 
                 linearized_plant: LinearSystem, # The continuous time linearization of the plant dynamics
                 x_0: np.array, # The ppoint about which we linearized the plant
                 x_init: np.array, # The initial state estimate
                 position_noise_covariance: np.array, 
                 acceleration_noise_covariance: np.array, 
                 dt_update: float = 1e-3):
        n_sense = np.shape(position_noise_covariance)[0] + np.shape(acceleration_noise_covariance)[0]
        n_est_states = len(x_0)
        n_actuators = 0
        super().__init__(n_sense, n_est_states, n_actuators, dt_update)

        # Initialize the state estimate, state covariance, and process noise covariance
        self.x_0 = x_0 # The point about which we linearized the plant
        self.x_k = x_init - x_0 # Initial state estimate
        self.P_k = 0.0001*np.eye(n_est_states) # Initial covariance (assumed small)
        
        # Extract the linearized plant matrices
        dap_dx = np.squeeze(linearized_plant.A()[2,:])
        datheta_dx = np.squeeze(linearized_plant.A()[3,:])
        dap_du = np.squeeze(linearized_plant.B()[2])
        datheta_du = np.squeeze(linearized_plant.B()[3])
        
        A_continuous_time = np.array([[0.0, 0.0, 1.0, 0.0], 
                                      [0.0, 0.0, 0.0, 1.0], 
                                      [0.0, 0.0, 0.0, 0.0],
                                      datheta_dx - datheta_du/dap_du*dap_dx])
        B_continuous_time = np.array([[0.0], [0.0], [1.0], [datheta_du/dap_du]])
        C_continuous_time = B_continuous_time.copy()

        # Discretize the continuous time matrices
        self.A = np.eye(n_est_states) + dt_update*A_continuous_time
        self.B = dt_update*B_continuous_time
        self.Q = dt_update**2*acceleration_noise_covariance[0][0]*C_continuous_time@C_continuous_time.T

        # Measurement noise covariance (adjust as needed)
        self.R = np.array(position_noise_covariance)*dt_update**2

        # Measurement matrix
        self.H = np.array([[0.0, 1.0, 0.0, 0.0]])

        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update, 0.0, self.UpdateEstimate)

    def UpdateEstimate(self, context: Context, state):
        # Get current sensor reading and previous acceleration value
        sensing_k = self.get_sensor_input_port().Eval(context)
        z_k = [sensing_k[0] - self.x_0[1]] # Make sure to map these into linearized states
        acceleration_k = [sensing_k[1]]

        # Prediction step: Propagate the state estimate and covariance forward
        x_k_prior = self.A @ self.x_k + self.B @ acceleration_k
        P_k_prior = self.A @ self.P_k @ self.A.T + self.Q

        # Correction step: Calculate Kalman gain and update the estimate and covariance
        K_k = P_k_prior @ self.H.T @ np.linalg.inv(self.H @ P_k_prior @ self.H.T + self.R)
        self.x_k = x_k_prior + np.squeeze(K_k @ (z_k - self.H @ x_k_prior))
        self.P_k = P_k_prior - np.squeeze(K_k @ self.H @ P_k_prior)


    def CalcObserverOutput(self, context: Context, output: BasicVector):
        # Reshape est_state to match the desired shape (2,)
        output.SetFromVector(np.squeeze(self.x_k + self.x_0))

class CPLQRController(Controller):
    # See http://underactuated.mit.edu/lqr.html for an overview on LQR
    def __init__(
        self,
        linearized_plant: LinearSystem,
        Q: np.ndarray = np.diag(np.array([10.0, 1.0, 1.0, 1.0])),
        R: np.ndarray = np.array([[1.0]]),
        max_force: np.ndarray = np.array([100.0]),
    ):
        K,_ = DiscreteTimeLinearQuadraticRegulator(linearized_plant.A(),
                                                   linearized_plant.B(),
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
        default=0.01,
        type=float,
        help="Add position sensor noise with given std (0.01 is a good start!)",
    )
    parser.add_argument(
        "-acc_sensor_noise",
        dest="acc_noise_std",
        default=0.01,
        type=float,
        help="Add acceleration sensor noise with given std (0.01 is a good start!)",
    )
    args = parser.parse_args()

    # Build the plant
    builder, plant, scene_graph = BuildCPPlant(dt=1e-3)
    x_0 = np.array([0., np.pi, 0., 0.])
    x_init = np.array([1., np.pi-0.5, 0., 0.])

    # Build the control system
    if args.cheater_observer:
        sensing = None
    else:
        sensing = CPPositionAndAccelerationSensor(position_noise_covariance=[[args.pos_noise_std**2]],
                                                  acceleration_noise_covariance=[[args.acc_noise_std**2]])

    linearized_plant_discrete_time = LinearizeCPPlant(plant, x_0)
    _,plant_continuous_time,_ = BuildCPPlant(dt = 0)
    linearized_plant_continuous_time = LinearizeCPPlant(plant_continuous_time, x_0)

    system = ControlSystem(
        builder,
        plant,
        scene_graph,
        sensing,
        CPKalmanObserver(linearized_plant_continuous_time,x_0 = x_0,x_init = x_init,position_noise_covariance=[[args.pos_noise_std**2]],acceleration_noise_covariance=[[args.acc_noise_std**2]]),
        CPLQRController(linearized_plant_discrete_time),
        CPTarget(),
    )
    # Simulate the control system
    log_data = system.Simulate(x_init,20, wait=False)

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

    a = 1
