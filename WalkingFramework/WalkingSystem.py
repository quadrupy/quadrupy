from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

import yaml

class WalkingSystem():
    def __init__(self, config_file: str, is_sim=True):
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
            if observer_class == 'WalkingObserver':
                from .observers.WalkingObserver import WalkingObserver
                self.observer = WalkingObserver(self.robot, config_dict=config_dict['observer'])
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
            self.observer.AddToBuilderAndConnect(builder, robot=self.robot)
        if self.controller is not None:
            self.controller.AddToBuilderAndConnect(builder, robot=self.robot, target=self.target, observer=self.observer)

        self.diagram = builder.Build()

        return
    
    def RunSystem(self, t_end: float):
        simulator = Simulator(self.diagram)
        simulator.Initialize()
        simulator.AdvanceTo(t_end)
        self.robot.ReplayRecording()

        return