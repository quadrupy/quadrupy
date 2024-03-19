from quadrupy.WalkingSystem import WalkingSystem
import matplotlib.pyplot as plt
import numpy as np
if __name__ == '__main__':
    config_file = './configs/Go2Walking.yaml'

    walking_system = WalkingSystem(config_file, is_sim=False, use_cheater_observer=False)

    walking_system.RunSystem(20.0, target_rate=1, ignore_error=True)
    walking_system.robot.cs = np.transpose(walking_system.robot.cs)
    walking_system.robot.ias = np.transpose(walking_system.robot.ias)
    print(walking_system.robot.ias.shape)
    fig, ax = plt.subplots(2,1)
    for i in range(0,len(walking_system.robot.cs)):
        ax[0].plot(walking_system.robot.cs[i], label=i)
    for i in range(0,len(walking_system.robot.ias)):
        ax[1].plot(walking_system.robot.ias[i], label=i)
    plt.show()
    # walking_system.PlotSystemGraph()

    input("Press Enter to continue...")