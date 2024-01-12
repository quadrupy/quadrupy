from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator

from WalkingFramework.WalkingSystem import WalkingSystem

if __name__ == '__main__':
    config_file = 'WalkingFramework/configs/Go2Walking.yaml'

    walking_system = WalkingSystem(config_file, is_sim=True)

    walking_system.RunSystem(10.0)

    input("Press Enter to continue...")