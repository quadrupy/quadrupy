from numpy.core.multiarray import array as array
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant, CoulombFriction
from pydrake.systems.framework import DiagramBuilder
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.geometry import HalfSpace, Box

import time
import numpy as np
import os

from .WalkingRobot import WalkingRobot

class Go2RobotSettings():
    def __init__(self, friction = 0.5, llc_dt = 0.001, controller_dt = 0.01):
        self.friction = friction
        self.llc_dt = llc_dt                    # The time step of the low level controllers (on the Go1 hardware this is fixed at 1e-3)
        self.controller_dt = controller_dt      # The time step of the high level controller, for walking 1e-2 is a good rate
    
    def OverwriteFromDict(self, config_dict:dict):
        if config_dict.__contains__('friction'):
            self.friction = config_dict['friction'] 
        if config_dict.__contains__('llc_dt'):
            self.llc_dt = config_dict['llc_dt']
        if config_dict.__contains__('controller_dt'):
            self.controller_dt = config_dict['controller_dt']

class Go2Robot(WalkingRobot):
    def __init__(self, is_sim=True, settings: Go2RobotSettings = None, config_dict: dict = None):
        self.is_sim = is_sim

        self.settings = settings
        if self.settings is None:
            self.settings = Go2RobotSettings()
        if config_dict is not None:
            self.settings.OverwriteFromDict(config_dict)

        builder = DiagramBuilder()
        plant: MultibodyPlant
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=self.settings.llc_dt)
        parser = Parser(plant)
        parser.AddModels(os.path.join(os.path.dirname(__file__), 'assets/Go2/go2_description.urdf'))

        # Drake doesn't import actuators correctly from the URDF, so we have to do it manually
        joint_actuation_limits = {'hip':23.7,'thigh':23.7,'calf':45.43} # These values come from the urdf
        actuation_limits = []
        ji = plant.GetJointIndices(plant.GetModelInstanceByName("go2_description"))
        for i in ji:
            j = plant.get_joint(i)
            if j.num_velocities() > 0:
                plant.AddJointActuator(j.name(),j)
                act_lim = np.inf
                for jtype in joint_actuation_limits.keys():
                    if jtype in j.name(): 
                        act_lim = joint_actuation_limits[jtype]
                actuation_limits.append(act_lim)
        actuation_limits = np.array(actuation_limits)

        # Add floor
        dir_ground = np.array([0,0.0,1.0])
        p_ground = np.zeros((3,1))
        X_WG = RigidTransform(HalfSpace.MakePose(dir_ground,p_ground))
        #   Define friction
        friction = CoulombFriction(static_friction=self.settings.friction,dynamic_friction=self.settings.friction)
        #   Register ground contact
        plant.RegisterCollisionGeometry(plant.world_body(),X_WG,HalfSpace(),'ground',friction)
        plant.RegisterVisualGeometry(plant.world_body(),RigidTransform([0.,0.,-0.5]),Box(20,20,1),'ground',[0.9,0.9,0.9,1.])

        plant.Finalize()
        plant.SetDefaultPositions([1.,0.,0.,0., 0.,0.,0.325, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2, 0.,np.pi/4,-np.pi/2])

        imu_body = plant.GetBodyByName("imu")
        root_body = plant.GetBodyByName("base")

        contacts = [(plant.GetFrameByName("FL_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("FL_foot"))),
                    (plant.GetFrameByName("FR_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("FR_foot"))),
                    (plant.GetFrameByName("RL_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("RL_foot"))),
                    (plant.GetFrameByName("RR_foot"),np.zeros([1,3]),plant.GetCollisionGeometriesForBody(plant.GetBodyByName("RR_foot")))]
        WalkingRobot.__init__(self,builder,plant,scene_graph,imu_body,root_body,contacts,actuation_limits,dt=self.settings.controller_dt,is_sim=is_sim)
        
        # Useful variables for IK
        self.foot_names = ['FL','FR','RL','RR']
        self.shoulder_positions = np.array([[sign[0]*0.1934, sign[1]*0.0465, 0.] for sign in [[1,1],[1,-1],[-1,1],[-1,-1]]]) # note left is positive y
        
        self.FOOT_FORCE_THRES = 25

        # to save for plotting purposes
        self.joint_pos = []
        self.joint_vels = []
        self.joint_torques = []
        self.imu_accels = []
        self.imu_ang_vels = []
        self.contact_states = []

        # stand example
        self.is_first_run = True
        self.duration_1 = 500
        self.duration_2 = 500
        self.duration_3 = 1000
        self.duration_4 = 900
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0
        self.target_pos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self.target_pos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self.target_pos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]
        self.done = False
        self.kp = 60.0
        self.kd = 5.0
        self.time_consume = 0
        self.rate_count = 0
        self.sin_count = 0
        self.motion_time = 0
        self.dt = 0.002
        self.start_pos = [0]*12
        self.t0 = time.time()

    def stand_ex(self):
        if self.done:
            return
        if self.percent_4 == 1:
            print("Finished example")
            self.done = True
            return
        self.motion_time += 1
        if self.motion_time >= 500:
            if (self.is_first_run):
                for i in range(0, 12):
                    self.start_pos = np.array(self.hardware_robot.q())
                self.is_first_run = False
            if self.percent_1 < 1:
                self.percent_1 += 1/self.duration_1
                if self.percent_1 > 1:
                    self.percent_1 = 1
                q = [0]*12
                dq = [0]*12
                kp = [0]*12
                kd = [0]*12
                tau = [0]*12
                for i in range(0, 12):
                    q[i] = (1-self.percent_1)*self.start_pos[i] + self.percent_1*self.target_pos_1[i]
                    dq[i] = 0
                    kp[i] = self.kp
                    kd[i] = self.kd
                    tau[i] = 0
                self.hardware_robot.set_motor_cmd(q, dq, kp, kd, tau)
            if self.percent_1 == 1 and self.percent_2 < 1:
                self.percent_2 += 1/self.duration_2
                if self.percent_2 > 1:
                    self.percent_2 = 1
                q = [0]*12
                dq = [0]*12
                kp = [0]*12
                kd = [0]*12
                tau = [0]*12
                for i in range(0, 12):
                    q[i] = (1-self.percent_2)*self.target_pos_1[i] + self.percent_2*self.target_pos_2[i]
                    dq[i] = 0
                    kp[i] = self.kp
                    kd[i] = self.kd
                    tau[i] = 0
                self.hardware_robot.set_motor_cmd(q, dq, kp, kd, tau)
            if self.percent_1 == 1 and self.percent_2 == 1 and self.percent_3 < 1:
                self.percent_3 += 1/self.duration_3
                if self.percent_3 > 1:
                    self.percent_3 = 1
                q = [0]*12
                dq = [0]*12
                kp = [0]*12
                kd = [0]*12
                tau = [0]*12
                for i in range(0, 12):
                    q[i] = self.target_pos_2[i]
                    dq[i] = 0
                    kp[i] = self.kp
                    kd[i] = self.kd
                    tau[i] = 0
                self.hardware_robot.set_motor_cmd(q, dq, kp, kd, tau)
            if self.percent_1 == 1 and self.percent_2 == 1 and self.percent_3 == 1 and self.percent_4 < 1:
                self.percent_4 += 1/self.duration_4
                if self.percent_4 > 1:
                    self.percent_4 = 1
                q = [0]*12
                dq = [0]*12
                kp = [0]*12
                kd = [0]*12
                tau = [0]*12
                for i in range(0, 12):
                    q[i] = (1 - self.percent_4) * self.target_pos_2[i] + self.percent_4 * self.target_pos_3[i]
                    dq[i] = 0
                    kp[i] = self.kp
                    kd[i] = self.kd
                    tau[i] = 0
                self.hardware_robot.set_motor_cmd(q, dq, kp, kd, tau)
            self.hardware_robot.set_crc()
            self.hardware_robot.write()

    def HardwareUpdate(self):
            # Advance simulator by controller dt
        # self.simulator.AdvanceTo(self.sim_ctx.get_time()+self.dt)
        target_pos = self.plant.GetDefaultPositions()[7:]
        # Send actuation commands to the robot
        # target_pos[0] /= 2
        # target_pos[1] /= 2
        # target_pos[2] /= 2 
        t = time.time()
        if t-self.t0 < 10:
            kp = min(60, 6*(t-self.t0))
            self.hardware_robot.set_motor_cmd(list(target_pos), [0]*12, [kp]*12, [5]*12, [0]*12)
            self.sensing_data.contact_state = [1]*4
            print("standby")
        else:
            self.hardware_robot.set_motor_cmd(list(self.actuation_data.q), list(self.actuation_data.dq), list(self.actuation_data.Kp), list(self.actuation_data.Kd), list(self.actuation_data.tau))
            self.sensing_data.contact_state = self.hardware_robot.foot_force()
            for i in range(0, self.num_contacts):
                self.sensing_data.contact_state[i] = 1 if self.sensing_data.contact_state[i] >= self.FOOT_FORCE_THRES else 0
        self.hardware_robot.set_crc()
        self.hardware_robot.write()

        # set sensing data
        self.sensing_data.joint_pos = self.hardware_robot.q()
        self.sensing_data.joint_vel = self.hardware_robot.dq()
        self.sensing_data.joint_torque = self.hardware_robot.tau()
        self.sensing_data.imu_acc = self.hardware_robot.imu_accel()
        self.sensing_data.imu_ang_vel = self.hardware_robot.imu_ang_vel()
        self.sensing_data.time_stamp = time.time() - self.t0
        # self.sensing_data.time_stamp = 0
        # print(vars(self.actuation_data))
        # print(self.sensing_data.time_stamp)
        # for plotting
        self.imu_accels.append(self.sensing_data.imu_acc)
        self.imu_ang_vels.append(self.sensing_data.imu_ang_vel)
        self.contact_states.append(self.sensing_data.contact_state)

    def GetShoulderPosition(self, foot_idx: int) -> np.array:
        return self.shoulder_positions[foot_idx]

    def CalcFootIK(self, foot_idx: int, foot_pos_robot_frame: np.array, foot_rot: RotationMatrix = None):
        lH = 0.095
        lT = 0.213 # length of both thigh and shank
        
        footPos = foot_pos_robot_frame - self.shoulder_positions[foot_idx]
        # footPos = [xF,yF,zF]
        # xF is forward/backward coordinate from shoulder
        # yF is side-to-side coordinate from shoulder
        # zF is vertical coordinate from shoulder
        # Start with hip angle
        xF = np.clip(footPos[0],-1.,1.)
        yF = np.clip(footPos[1],-1.,1.)
        zF = np.clip(footPos[2],-1.,-1e-3)
        mirror = False
        if (self.foot_names[foot_idx] == 'FR') or (self.foot_names[foot_idx] == 'RR'):
            mirror = True
            yF *= -1
        yF += lH # Shift yF to put our coordinate frame at the leg root

        q_hip = -np.arcsin(np.clip((-np.sqrt(np.clip(yF**2 + zF**2 - lH**2,0.,np.inf))*yF - lH*zF)/(yF**2 + zF**2),-1,1))
        y_hip = lH*(np.cos(q_hip))
        z_hip = lH*np.sin(q_hip)
        l_leg = np.clip(np.sqrt(xF**2 + (yF - y_hip)**2 + (zF - z_hip)**2),1e-3,2*lT-1e-3)
        q_knee = -2*np.arccos(l_leg/(2*lT))
        q_thigh = np.arcsin(-xF/l_leg) - q_knee/2

        if mirror:
            q_hip *= -1

        return np.array([q_hip,q_thigh,q_knee])



    