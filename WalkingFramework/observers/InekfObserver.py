from numpy.core.multiarray import array as array
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction
from pydrake.multibody.tree import ModelInstanceIndex, SpatialInertia
from pydrake.systems.framework import Context, DiagramBuilder
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.common.eigen_geometry import Quaternion
from pydrake.geometry import Meshcat, MeshcatVisualizer, Box, MeshcatVisualizerParams, Role, Sphere

import numpy as np

from ..bindings.lib import inekf_py as inekf

from .WalkingObserver import WalkingObserver
from ..robots.WalkingRobot import WalkingRobot, SensorData

class InekfObserverSettings():
    def __init__(self, encoder_noise = 1*np.pi/180, gyro_noise = 10., gyro_bias = 0.0, accel_noise = 10., accel_bias = 0.0, contact_noise = 10., foot_offset = 0.0, add_visualizer = False):
        self.encoder_noise = encoder_noise
        self.gyro_noise = gyro_noise
        self.gyro_bias = gyro_bias
        self.accel_noise = accel_noise
        self.accel_bias = accel_bias
        self.contact_noise = contact_noise
        self.foot_offset = foot_offset
        self.add_visualizer = add_visualizer
    
    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('encoder_noise'):
            self.encoder_noise = config_dict['encoder_noise']
        if config_dict.__contains__('gyro_noise'):
            self.gyro_noise = config_dict['gyro_noise']
        if config_dict.__contains__('gyro_bias'):
            self.gyro_bias = config_dict['gyro_bias']
        if config_dict.__contains__('accel_noise'):
            self.accel_noise = config_dict['accel_noise']
        if config_dict.__contains__('accel_bias'):
            self.accel_bias = config_dict['accel_bias']
        if config_dict.__contains__('contact_noise'):
            self.contact_noise = config_dict['contact_noise']
        if config_dict.__contains__('foot_offset'):
            self.foot_offset = config_dict['foot_offset']
        if config_dict.__contains__('add_visualizer'):
            self.add_visualizer = config_dict['add_visualizer']
        

class InekfObserver(WalkingObserver):
    def __init__(self, robot:WalkingRobot, settings: InekfObserverSettings = None, config_dict: dict = None):
        WalkingObserver.__init__(self, robot.plant.num_actuators(), robot.num_contacts)
        self.robot = robot
        self.observer_context = self.robot.plant.CreateDefaultContext()
        self.prev_sensor_data = SensorData(self.robot.plant.num_actuators(),self.robot.num_contacts)

        # Parse settings
        self.settings = settings
        if self.settings is None:
            self.settings = InekfObserverSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        # Initialize the robot parameters
        initRootPos = RigidTransform(Quaternion(robot.plant.GetDefaultPositions()[:4]), robot.plant.GetDefaultPositions()[4:7])
        self.imu_to_root = self.robot.plant.CalcRelativeTransform(self.observer_context,self.robot.imu_frame,self.robot.root_frame)
        initIMUPos:RigidTransform = initRootPos@(self.imu_to_root.inverse())

        # Initialize InEKF
        #   Encoder covariance matrix
        self.Cov_e =  self.settings.encoder_noise*np.eye(self.robot.num_act)
        #   Noise parameters
        inekf_noise = inekf.NoiseParams()
        inekf_noise.setGyroscopeNoise(self.settings.gyro_noise)
        inekf_noise.setAccelerometerNoise(self.settings.accel_noise)
        inekf_noise.setGyroscopeBiasNoise(self.settings.gyro_bias)
        inekf_noise.setAccelerometerBiasNoise(self.settings.accel_bias)
        inekf_noise.setContactNoise(self.settings.contact_noise)
        #   Initial state
        inekf_state = inekf.RobotState()
        inekf_state.setRotation(initIMUPos.rotation().matrix())
        inekf_state.setVelocity(np.zeros(3))
        inekf_state.setPosition(initIMUPos.translation())
        inekf_state.setGyroscopeBias(np.zeros(3))
        inekf_state.setAccelerometerBias(np.zeros(3))
        #   Kinematics object
        self.inekf_kinematics = [inekf.Kinematics(i,np.eye(4),np.eye(6)) for i in range(self.robot.num_contacts)]
        #   Inekf filter
        self.inekf_filter = inekf.InEKF(inekf_state,inekf_noise)

        # Initialize the visualizer
        if self.settings.add_visualizer:
            self.InitVisualizer()

                
    def InitVisualizer(self):
        meshcat = self.robot.meshcat

        # Get equivalent box for the torso
        body_spatial_inertia = self.robot.root_body.CalcSpatialInertiaInBodyFrame(self.robot.plant.CreateDefaultContext())
        body_principal_moments = body_spatial_inertia.Shift(body_spatial_inertia.get_com()).CalcRotationalInertia().CalcPrincipalMomentsAndAxesOfInertia()
        A = 1/12*6.92*np.array([[0,1,1],[1,0,1],[1,1,0]])
        box_lengths = np.sqrt(np.linalg.inv(A)@body_principal_moments[0])
        box_origin = RigidTransform(body_principal_moments[1],body_spatial_inertia.get_com())
        box_shape = Box(*box_lengths)

        vis_diagram_builder = DiagramBuilder()
        self.vis_plant, self.vis_sg = AddMultibodyPlantSceneGraph(vis_diagram_builder, time_step=0)
        self.vis_plant:MultibodyPlant
        self.vis_plant.set_name("observervis_plant")
        self.vis_sg.set_name("observervis_scene_graph")


        body_name = "observer_torso"
        plant_body = self.vis_plant.AddRigidBody(body_name,ModelInstanceIndex(1),SpatialInertia())
        self.vis_plant.RegisterVisualGeometry(plant_body,box_origin,box_shape,body_name,[0.5,0.5,0.5,1.0])

        foot_shape = Sphere(0.025)
        for i in range(self.robot.num_contacts):
            foot_name = f"observer_foot_projection_{i}"
            plant_foot = self.vis_plant.AddRigidBody(foot_name,ModelInstanceIndex(1),SpatialInertia())
            self.vis_plant.RegisterVisualGeometry(plant_foot,RigidTransform(),foot_shape,foot_name,[0.5,0.5,0.5,1.0])

        self.vis = MeshcatVisualizer.AddToBuilder(vis_diagram_builder, self.vis_sg, meshcat,
                                                MeshcatVisualizerParams(role=Role.kPerception, prefix="mpc_vis"))
        
        self.vis_plant.Finalize()
        self.vis_diagram = vis_diagram_builder.Build()
        self.vis_diagram_ctx = self.vis_diagram.CreateDefaultContext()
        self.vis_plant_ctx = self.vis_plant.GetMyContextFromRoot(self.vis_diagram_ctx)
        self.vis_ctx = self.vis.GetMyContextFromRoot(self.vis_diagram_ctx)
        self.vis_pos = self.vis_plant.GetDefaultPositions()

    def UpdateVisualizer(self,body_position,foot_positions):
        self.vis_pos[:7] = body_position
        for i in range(self.robot.num_contacts):
            self.vis_pos[7*(i+1)+4:7*(i+2)] = foot_positions[i]
        self.vis_plant.SetPositions(self.vis_plant_ctx, self.vis_pos)
        self.vis.ForcedPublish(self.vis_ctx)

    def CalcOutput(self, context: Context, output: AbstractValue):
        sensor_data:SensorData = self.sensing_in.Eval(context)
        if np.linalg.norm(sensor_data.joint_pos) < 1e-6:
            output.set_value(np.zeros(self.n_state))
            return

        plant_ctx = self.observer_context
        q = np.hstack([[1.,0.,0.,0.,0.,0.,0.],sensor_data.joint_pos])
        dq = np.hstack([[0.,0.,0.,0.,0.,0.],sensor_data.joint_vel])
        self.robot.plant.SetPositions(plant_ctx,q)

        # Calculate the kinematics properties of the contact points
        #   First the relative positions of the feet in the IMU frame
        footPos = [self.robot.plant.CalcPointsPositions(plant_ctx,fc[0],np.zeros(3),self.robot.imu_frame).T[0] for fc in self.robot.contacts]
        #   Then the jacobian of the feet with respect to joint angles in the IMU frame
        footPosJac = [np.array(self.robot.plant.CalcJacobianPositionVector(plant_ctx,fc[0],np.zeros(3),self.robot.imu_frame,self.robot.imu_frame))[:,7:] for fc in self.robot.contacts]
        footRotJac = np.zeros([3,self.robot.num_act]) # Assume foot rotations aren't used in the estimate (with point contacts this is a good assumption)
        footJac = [np.vstack([footRotJac,fJ]) for fJ in footPosJac]
        #   Finally the covariance of the foot positions in the IMU frame due to encoder noise
        footCov = [fJ@self.Cov_e@fJ.T for fJ in footJac]
        
        footInContact = sensor_data.contact_state
        
        dt = sensor_data.time_stamp - self.prev_sensor_data.time_stamp
        if (dt >= 0.0001) and (dt < 0.100):
            imu_data_prev = np.hstack([self.prev_sensor_data.imu_ang_vel,self.prev_sensor_data.imu_acc])
            
            self.inekf_filter.Propagate(imu_data_prev, dt)
            self.inekf_filter.setContacts([(i,footInContact[i]) for i in range(4)])
            
            for i in range(self.robot.num_contacts):
                self.inekf_kinematics[i].pose = np.vstack([np.hstack([np.eye(3),np.array([footPos[i]]).T]),np.array([[0.,0.,0.,1.]])]).copy()
                self.inekf_kinematics[i].covariance = footCov[i].copy()
                
            self.inekf_filter.CorrectKinematics(self.inekf_kinematics)

        imu_pos = RigidTransform(RotationMatrix(self.inekf_filter.getState().getRotation()),self.inekf_filter.getState().getPosition())
        root_pos:RigidTransform = imu_pos@self.imu_to_root
        imu_vel = self.inekf_filter.getState().getVelocity()
        root_vel = self.imu_to_root.rotation()@imu_vel
        imu_ang_vel = sensor_data.imu_ang_vel
        root_ang_vel = self.imu_to_root.rotation()@imu_ang_vel

        # Shift the body height such that the lowest stance foot is at z = 0
        world_foot_pos = [imu_pos@RigidTransform(RotationMatrix(np.eye(3)),fp).translation() for fp in footPos]
        shift_height = min([fp[2] for fp in world_foot_pos]) - self.settings.foot_offset

        q[:4] = root_pos.rotation().ToQuaternion().wxyz()
        q[4:7] = root_pos.translation() + [0., 0., -shift_height]
        dq[:3] = root_ang_vel
        dq[3:6] = root_vel

        if self.settings.add_visualizer:
            world_foot_pos =  [fp + [0., 0., -shift_height] for fp in world_foot_pos]
            self.UpdateVisualizer(q[:7],foot_positions=world_foot_pos)

        self.prev_sensor_data.copy_from(sensor_data)
        output.set_value(np.hstack([q,dq]))
        return

