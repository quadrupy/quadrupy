from quadrupy.WalkingSystem import WalkingSystem
import matplotlib.pyplot as plt
import numpy as np
import logging 

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    
    config_file = './configs/Go2Walking.yaml'
    walking_system = WalkingSystem(config_file, is_sim=False, use_cheater_observer=False)
    walking_system.RunSystem(np.inf, target_rate=1, ignore_error=False)
