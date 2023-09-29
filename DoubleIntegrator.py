from pydrake.geometry import SceneGraph
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder, BasicVector, Context

from ControlFramework import Sensing, Observer, Controller, Target, ControlSystem

import numpy as np
import matplotlib.pyplot as plt
import sys
import argparse

def BuildDIPlant()->'tuple[DiagramBuilder,MultibodyPlant,SceneGraph]':
    builder = DiagramBuilder()
    plant:MultibodyPlant
    plant,scene_graph = AddMultibodyPlantSceneGraph(builder,1e-3)
    parser = Parser(plant)
    parser.AddModels('DoubleIntegrator.urdf')
    plant.Finalize()
    return builder,plant,scene_graph

class DIPositionSensor(Sensing):
    def __init__(self,selection_matrix:np.ndarray=np.array([[1.,0.]]),
                      noise_covariance:np.array=None):
        self.selection_matrix = selection_matrix
        self.n_outputs = selection_matrix.shape[0]
        n_states = selection_matrix.shape[1]
        if noise_covariance is None:
            noise_covariance = np.zeros([self.n_outputs,self.n_outputs])
        self.noise_covariance = noise_covariance

        super().__init__(n_states,self.n_outputs)

    def CalcSensorData(self,context:Context,output:BasicVector):
        robot_state = self.get_state_input_port().Eval(context)
        noise = np.random.multivariate_normal(np.zeros(self.n_outputs),self.noise_covariance)

        output.SetFromVector(self.selection_matrix@robot_state + noise)

class DINaiveObserver(Observer):
    def __init__(self,n_sense=1,n_est_states=2,dt_update:float=1e-3):
        super().__init__(n_sense,n_est_states)

        # Initialize our stored variables and position estimate
        self.prev_pos = 0.
        self.prev_time = -1.e-3
        self.est_state = np.zeros(2)

        # Periodically call UpdateEstimate to recompute the finite difference
        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update,0.,self.UpdateEstimate)

    def UpdateEstimate(self,context:Context,state):
        # Get the current simlator time
        curr_time = context.get_time()
        # Read in the sensor signal from the input port, and use this as our position estimate
        self.est_state[0] = self.get_sensor_input_port().Eval(context)
        # use the previous stored position estimate and time to do a finite difference for velocity
        self.est_state[1] = (self.est_state[0] - self.prev_pos)/(curr_time - self.prev_time)

        # Update the stored position and time
        self.prev_pos = self.est_state[0]
        self.prev_time = curr_time

    def CalcObserverOutput(self,context:Context,output:BasicVector):
        output.SetFromVector(self.est_state)

class DIKalmanObserver(Observer):
    def __init__(self,n_sense=1,n_est_states=2,dt_update:float=1e-3):
        # Initialize the observer here
        self.est_state = np.zeros(2)

        super().__init__(n_sense,n_est_states)
        self.DeclarePeriodicUnrestrictedUpdateEvent(dt_update,0.,self.UpdateEstimate)
    
    def UpdateEstimate(self,context:Context,state):
        # Update the estimated state here
        pass

    def CalcObserverOutput(self, context: Context, output: BasicVector):
        output.SetFromVector(self.est_state)

class DIController(Controller):
    def __init__(self,K:np.ndarray = np.array([[100.,50.]]),max_force:np.ndarray = np.array([100.])):
        n_states = K.shape[1]
        n_outputs = K.shape[0]
        self.K = K
        self.max_force = max_force
        super().__init__(n_states,n_states,n_outputs)

    def CalcControllerOutput(self, context: Context, output: BasicVector):
        est_robot_state = self.get_state_input_port().Eval(context)
        target_state = self.get_target_input_port().Eval(context)
        act_out = np.clip(self.K@(target_state - est_robot_state),-self.max_force,self.max_force)
        output.SetFromVector(act_out)

class DITarget(Target):
    def __init__(self, omega:float = 1.):
        n_targets = 2
        self.omega = omega
        super().__init__(n_targets)

    def CalcTargetOutputs(self, context: Context, output: BasicVector):
        t = context.get_time()
        des_pos = np.cos(self.omega*t) - 1
        des_vel = -self.omega*np.sin(self.omega*t)
        output.SetAtIndex(0,des_pos)
        output.SetAtIndex(1,des_vel)


if __name__ == '__main__':
    # Parse command line arguments 
    parser = argparse.ArgumentParser(description='Simulate a double integrator control system')
    parser.add_argument('-sensor_noise',dest='noise_std',default=0.,type=float,help='Add sensor noise with given std (0.01 is a good start!)')
    args = parser.parse_args()
    
    # Build the plant
    builder,plant,scene_graph = BuildDIPlant()
    # Build the control system
    noise = sys.argv[0]
    system = ControlSystem(builder,plant,scene_graph,DIPositionSensor(noise_covariance=[[args.noise_std**2]]),DINaiveObserver(),DIController(),DITarget())
    # Simulate the control system
    log_data = system.Simulate(np.array([0.,0.]),20,wait=False)
    
    # Parse the logged data and plot the tracking performance
    time = np.array(log_data['time'])
    true_state = np.array(log_data['plant_data'])
    ref_state = np.array(log_data['target_data'])
    est_state = np.array(log_data['observer_data'])

    plt.figure()
    plt.plot(time,true_state[:,0],label='Actual',zorder=2)
    plt.plot(time,est_state[:,0],'--',label='Estimated',zorder=1)
    plt.plot(time,ref_state[:,0],':',label='Desired',zorder=3)
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()

    plt.figure()
    plt.plot(time,true_state[:,1],label='Actual',zorder=2)
    plt.plot(time,est_state[:,1],'--',label='Estimated',zorder=1)
    plt.plot(time,ref_state[:,1],':',label='Desired',zorder=3)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.show()
    
