from quadrupy.WalkingSystem import WalkingSystem

if __name__ == '__main__':
    config_file = './configs/Go2Walking.yaml'

    walking_system = WalkingSystem(config_file, is_sim=True, use_cheater_observer=False)

    walking_system.RunSystem(1000.0, target_rate=1, ignore_error=True)

    # walking_system.PlotSystemGraph()

    input("Press Enter to continue...")