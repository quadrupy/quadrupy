from quadrupy.WalkingSystem import WalkingSystem
import matplotlib.pyplot as plt
import numpy as np
import logging 

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    config_file = './configs/Go2Walking.yaml'

    walking_system = WalkingSystem(config_file, is_sim=True, use_cheater_observer=False, telemetry_url="ws://localhost:3000/api/live/push/go2")

    walking_system.RunSystem(1000.0, target_rate=1, ignore_error=True)
    walking_system.robot.contact_states = np.transpose(walking_system.robot.contact_states)
    walking_system.robot.imu_accels = np.transpose(walking_system.robot.imu_accels)
    fig, ax = plt.subplots(2,1)
    for i in range(0,len(walking_system.robot.contact_states)):
        # ax[0].plot(walking_system.robot.contact_states[i], label=i)
        ax[0].plot(walking_system.robot.contact_states[i], label=i)
    for i in range(0,len(walking_system.robot.imu_accels)):
        ax[1].plot(walking_system.robot.imu_accels[i], label=i)
    plt.show()
    # walking_system.PlotSystemGraph()

    input("Press Enter to continue...")