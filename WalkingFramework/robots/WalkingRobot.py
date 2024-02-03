from pydrake.systems.framework import LeafSystem, Context, DiagramBuilder, BasicVector
from pydrake.multibody.plant import MultibodyPlant
from pydrake.common.value import AbstractValue
from pydrake.multibody.tree import BodyFrame, Body, JointIndex
from pydrake.systems.analysis import Simulator
from pydrake.geometry import GeometryId, SceneGraph, Meshcat, MeshcatVisualizer, Role, MeshcatVisualizerParams
from pydrake.math import RotationMatrix
import numpy as np

class LLCActuationCommand():
    # Low-level controller actuation command. This is the input to the low level controllers on the robot
    def __init__(self, n_act) -> None:
        self.n_act = n_act
        self.q = np.zeros(n_act)
        self.dq = np.zeros(n_act)
        self.tau = np.zeros(n_act)
        self.Kp = np.zeros(n_act)
        self.Kd = np.zeros(n_act)

    def copy_from(self, llc_in: "LLCActuationCommand"):
        np.copyto(self.q,llc_in.q)
        np.copyto(self.dq,llc_in.dq)
        np.copyto(self.tau,llc_in.tau)
        np.copyto(self.Kp,llc_in.Kp)
        np.copyto(self.Kd,llc_in.Kd)

class SimLLC(LeafSystem):
    # Simulated version of the low level controller being run on the hardware robot
    def __init__(self,n_act,actuation_limits=None):
        LeafSystem.__init__(self)
        self.n_act = n_act
        if actuation_limits is None:
            actuation_limits = np.inf*np.ones(n_act)
        self.actuation_limits = actuation_limits

        # Set this manually before each simulation step
        self.llc_in = LLCActuationCommand(n_act)

        # Use this to keep track of the output torque
        self.output_torque = np.zeros(n_act)

        self.state_in = self.DeclareVectorInputPort('joint_qdq',2*n_act+13)
        self.torque_out = self.DeclareVectorOutputPort('torque_out',n_act,self.CalcTorque)

    def CalcTorque(self,context: Context,output: BasicVector):
        input_state = self.state_in.Eval(context)
        q = input_state[7:7+self.n_act]
        dq = input_state[7+self.n_act+6:]

        output_torque = self.llc_in.Kp*(self.llc_in.q-q) + self.llc_in.Kd*(self.llc_in.dq-dq) + self.llc_in.tau
        output_torque = np.clip(output_torque,-self.actuation_limits,self.actuation_limits)

        self.output_torque = output_torque
        output.set_value(output_torque)

    def SetLLCCommand(self,llc_in: LLCActuationCommand):
        self.llc_in.copy_from(llc_in)

class SensorData():
    # Sensor data from the robot
    def __init__(self, n_joints, n_contacts) -> None:
        self.n_joints = n_joints
        self.n_contacts = n_contacts
        self.joint_pos = np.zeros(n_joints)
        self.joint_vel = np.zeros(n_joints)
        self.joint_torque = np.zeros(n_joints)
        self.imu_acc = np.zeros(3)
        self.imu_ang_vel = np.zeros(3)
        self.contact_state = np.zeros(n_contacts)

    def copy_from(self, sensor_data_in: "SensorData"):
        np.copyto(self.joint_pos,sensor_data_in.joint_pos)
        np.copyto(self.joint_vel,sensor_data_in.joint_vel)
        np.copyto(self.joint_torque,sensor_data_in.joint_torque)
        np.copyto(self.imu_acc,sensor_data_in.imu_acc)
        np.copyto(self.imu_ang_vel,sensor_data_in.imu_ang_vel)
        np.copyto(self.contact_state,sensor_data_in.contact_state)

class WalkingRobot(LeafSystem):
    def __init__(self,builder:DiagramBuilder,                                            # DiagramBuilder object containing the reference plant  
                      reference_plant:MultibodyPlant,                                    # Robot plant, used to keep track of robot model parameters 
                      scene_graph:SceneGraph,                                            # Scene graph, used to keep track of robot geometry
                      imu_body:Body,                                                     # IMU body, used to contextualize IMU measurements
                      root_body:Body,                                                    # Root body, used to contextualize robot state
                      contacts:'list[tuple[BodyFrame,list[np.array],list[GeometryId]]]', # List of foot frames and list of contact points in each frame
                      actuation_limits:np.array = None,                                  # Array of max absolute joint torque for each actuator (nu array)
                      dt:float = 1e-3,                                                   # Time step between control data updates
                      is_sim:bool = True,                                                # Flag to indicate whether this is a simulation or hardware
                      sim_initial_state:np.array = None,                                 # Initial state for the simulator. If None, use the default state from the URDF
                      meshcat = None):                                                   # Meshcat object for visualization, if None, will create a new meshcat object
        LeafSystem.__init__(self)
        self.plant = reference_plant
        self.scene_graph = scene_graph
        self.imu_body = imu_body 
        self.imu_frame = imu_body.body_frame()
        self.root_body = root_body
        self.root_frame = root_body.body_frame()
        self.contacts = contacts
        self.num_contacts = sum([len(c[1]) for c in contacts])
        self.actuation_limits = actuation_limits
        self.num_act = reference_plant.num_actuators()
        self.dt = dt
        self.is_sim = is_sim
        
        self.actuation_data = LLCActuationCommand(self.num_act)
        self.sensing_data = SensorData(reference_plant.num_actuators(),self.num_contacts)
        self.cheater_state = np.zeros(reference_plant.num_multibody_states())

        nodep = set([self.time_ticket()])
        
        self.DeclarePeriodicUnrestrictedUpdateEvent(dt,0.0,self.PeriodicUpdate)
        self.llc_actuation_in = self.DeclareAbstractInputPort('llc_actuation_in',AbstractValue.Make(LLCActuationCommand(self.num_act)))
        self.sensing_out = self.DeclareAbstractOutputPort('sensing_out',lambda: AbstractValue.Make(SensorData(self.num_act,self.num_contacts)),self.CalcSensing,prerequisites_of_calc=nodep)
        self.cheater_state_out = self.DeclareVectorOutputPort('cheater_state_out',reference_plant.num_multibody_states(),self.CalcCheaterState,prerequisites_of_calc=nodep)
        self.selection_matrix = self.plant.MakeActuationMatrix() # Selection matrix (nvxnu) maps from actuators to joints

        # Create an internal diagram for the plant and scene graph
        # In simulation, this will be what we use to simulate our physics
        # In hardware, this will just be used to visualize the robot motion
        if meshcat is None:
            meshcat = Meshcat()
        self.meshcat = meshcat   
        meshcat_params = MeshcatVisualizerParams(role=Role.kPerception, prefix="visual",publish_period=1/64.)
        self.meshcat_vis = MeshcatVisualizer.AddToBuilder(builder, self.scene_graph, meshcat, meshcat_params)
        if is_sim:
            # Add LLC controller for the simulated robot. For the hardware robot, this already happens on board
            self.sim_llc = SimLLC(self.num_act,self.actuation_limits)
            builder.AddSystem(self.sim_llc)
            builder.Connect(reference_plant.get_state_output_port(), self.sim_llc.state_in)
            builder.Connect(self.sim_llc.torque_out, reference_plant.get_actuation_input_port())

        self.plant_diagram = builder.Build()

        self.meshcat_vis.StartRecording()
        if is_sim:
            # Create the internal simulator
            self.simulator = Simulator(self.plant_diagram)
            self.simulator.Initialize()
            self.simulator.set_publish_every_time_step(True)
            self.simulator.AdvanceTo(0.0)
            self.sim_ctx = self.simulator.get_context()
            self.sim_plant_ctx = self.plant.GetMyContextFromRoot(self.sim_ctx)
            self.vis_ctx = self.meshcat_vis.GetMyContextFromRoot(self.sim_ctx)
            if sim_initial_state is not None:
                self.plant.SetPositions(self.sim_plant_ctx,sim_initial_state)
        else:
            self.vis_ctx = self.meshcat_vis.CreateDefaultContext()


    def PeriodicUpdate(self,context: Context, output: AbstractValue):
        # This function is responsible for sending motor commands to the simulator/robot and filling in the sensing data
        self.actuation_data.copy_from(self.llc_actuation_in.Eval(context))
        
        if self.is_sim:
            self.SimUpdate()
        else:
            self.HardwareUpdate()

        self.meshcat_vis.ForcedPublish(self.vis_ctx)

    def SimUpdate(self):
        # Send actuation commands to the simulator
        self.sim_llc.SetLLCCommand(self.actuation_data)

        # Advance simulator by controller dt
        self.simulator.AdvanceTo(self.sim_ctx.get_time()+self.dt)

        # Calculate the sensor data
        body_accelerations = self.plant.get_body_spatial_accelerations_output_port().Eval(self.sim_plant_ctx)
        body_velocities = self.plant.get_body_spatial_velocities_output_port().Eval(self.sim_plant_ctx)
        imu_rot:RotationMatrix = self.plant.CalcRelativeTransform(self.sim_plant_ctx,self.imu_frame,self.plant.world_frame()).rotation()
        imu_acc_world_frame = body_accelerations[self.imu_body.index()].translational()
        imu_ang_vel_world_frame = body_velocities[self.imu_body.index()].rotational()

        # Set the sensor output
        self.sensing_data.joint_pos = self.plant.GetPositions(self.sim_plant_ctx)[7:]
        self.sensing_data.joint_vel = self.plant.GetVelocities(self.sim_plant_ctx)[6:]
        self.sensing_data.imu_acc = imu_rot.inverse()@imu_acc_world_frame
        self.sensing_data.imu_ang_vel = imu_rot.inverse()@imu_ang_vel_world_frame
        self.sensing_data.joint_torque = self.sim_llc.output_torque

        self.cheater_state = self.plant.GetPositionsAndVelocities(self.sim_plant_ctx)

        return

    def ReplayRecording(self):
        # self.meshcat_vis.StopRecording()
        self.meshcat_vis.PublishRecording()

    def HardwareUpdate(self):
        # Send actuation commands to the robot and receive sensing data
        raise NotImplementedError
    
    def CalcSensing(self,context: Context,output: AbstractValue):
        output.set_value(self.sensing_data)

    def CalcCheaterState(self,context: Context,output: BasicVector):
        output.set_value(self.cheater_state)

    def GetDependentJoints(self,frame:BodyFrame):
        # Calculate the joint indices for the joints that are dependent on the given frame
        # This is useful for calculating the joint indices for the legs
        joint_indices = []
        true_ind = 0
        for i in range(self.plant.num_joints()):
            joint = self.plant.get_joint(JointIndex(i))
            if joint.num_velocities() != 1:
                continue
            joint_bodies = self.plant.GetBodiesKinematicallyAffectedBy([JointIndex(i)])
            for j in joint_bodies:
                if self.plant.get_body(j).body_frame().index() == frame.index():
                    joint_indices.append(true_ind)
                    break
            true_ind += 1
        return joint_indices

    def CalcFootIK(self,foot_idx:int,foot_pos_robot_frame:np.array,foot_rot:RotationMatrix=RotationMatrix())->np.array:
        # Calculate the joint positions for the given foot position and rotation
        # This is useful for calculating the joint positions for the legs
        raise NotImplementedError
    
    def GetShoulderPosition(self,foot_idx:int)->np.array:
        # Get the shoulder position for the given foot index
        # This is useful for calculating the joint positions for the legs
        raise NotImplementedError