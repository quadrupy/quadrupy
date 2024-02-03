# ObserverDesign

## Install
Recommend using a venv with python 3.8. Run:
```
pip install --upgrade pip
pip install -r requirements.txt
```

To learn more about drake, check out the documentation [here](https://drake.mit.edu/pydrake/index.html)

To compile the third party C++ libraries:
```
sudo apt install libeigen3-dev
cd WalkingFramework/third_party
cmake .
make all
```

## Run 

To run the walking framework, call:
```
run_go2_sim.py
```
with a logitech gamepad f310 plugged in. Press A on the gamepad to start and stop walking. Press B on the gamepad to kill the controller.

## Helpful docs

[go1 sdk manual](https://docs.trossenrobotics.com/unitree_go1_docs/_downloads/7a5296f31f42c1c274387504531932ba/go1_unitree_legged_sdk_manual.pdf)

To configure IP address in ubuntu:
sudo ifconfig eth0 down # eth0 is your PC Ethernet port
sudo ifconfig eth0 192.168.123.162/24
sudo ifconfig eth0 up
ping 192.168.123.161

[go2 sdk center](https://support.unitree.com/home/en/developer)
