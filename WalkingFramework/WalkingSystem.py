from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
import matplotlib.pyplot as plt
from pydrake.systems.drawing import plot_system_graphviz

import yaml

class WalkingSystem():
    def __init__(self, config_file: str, is_sim=True, use_cheater_observer=False):
        self.is_sim = is_sim
        with open(config_file) as file:
            config_dict = yaml.safe_load(file)

        self.robot = None
        if config_dict.__contains__('robot'):
            robot_class = config_dict['robot']['class_name']
            if robot_class == 'Go2Robot':
                from .robots.Go2Robot import Go2Robot
                self.robot = Go2Robot(is_sim=is_sim, config_dict=config_dict['robot'])
            else:
                raise NotImplementedError(f'Robot class {robot_class} not implemented')
        if self.robot is None:
            raise RuntimeError('No robot specified in config file')
        
        self.observer = None
        if config_dict.__contains__('observer'):
            observer_class = config_dict['observer']['class_name']
            if observer_class == 'InekfObserver':
                from .observers.InekfObserver import InekfObserver
                self.observer = InekfObserver(self.robot,config_dict=config_dict['observer'])
            else:
                raise NotImplementedError(f'Observer class {observer_class} not implemented')
            
        self.controller = None
        if config_dict.__contains__('controller'):
            controller_class = config_dict['controller']['class_name']
            if controller_class == 'QuadrupedWBC':
                from .controllers.QuadrupedWBC import QuadrupedWBC
                self.controller = QuadrupedWBC(self.robot, config_dict=config_dict['controller'])
            else:
                raise NotImplementedError(f'Controller class {controller_class} not implemented')

        self.target = None
        if config_dict.__contains__('target'):
            target_class = config_dict['target']['class_name']
            if target_class == 'JoystickInput':
                from .targets.WalkingTarget import JoystickInput
                self.target = JoystickInput(config_dict=config_dict['target'])
            else:
                raise NotImplementedError(f'Target class {target_class} not implemented')

        builder = DiagramBuilder()
        builder.AddSystem(self.robot)
        if self.target is not None:
            builder.AddSystem(self.target)
        if self.observer is not None:
            self.observer.AddToBuilderAndConnect(builder, robot=self.robot, forced_update=use_cheater_observer)
        if self.controller is not None:
            if use_cheater_observer:
                self.controller.AddToBuilderAndConnect(builder, robot=self.robot, target=self.target, observer=None)
            else:
                self.controller.AddToBuilderAndConnect(builder, robot=self.robot, target=self.target, observer=self.observer)

        self.diagram = builder.Build()

        return
    
    def RunSystem(self, t_end: float, target_rate=1.0, ignore_error=True):
        simulator = Simulator(self.diagram)
        simulator.Initialize()
        simulator.set_target_realtime_rate(target_rate)
        try:
            simulator.AdvanceTo(t_end)
        except:
            if not ignore_error:
                raise
        self.robot.ReplayRecording()
        if self.observer is not None:
            self.observer.ReplayRecording()

        return
    
    def PlotSystemGraph(self):
        plt.figure()
        plot_system_graphviz(self.diagram)
        plt.show()