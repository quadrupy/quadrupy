<p align="center"> 
  <img width="300" src="https://github.com/quadrupy/quadrupy/assets/43913902/7cb6a98e-c395-4e9b-a63d-dbc4c1aaddde" />
</p>
<h3 align="center">Quadrupy: A Quadrupedal Development Platform</h3>


## Install
Recommend using a venv with python 3.11. Run:
```
pip install -e .
```

Set up live telemetry with Grafana, replacing `GRAFANA_SERVICE_ACCOUNT_TOKEN` with an admin service account API token:
```
cat <<EOT >> .env
GRAFANA_URL="localhost:3000"
GRAFANA_SERVICE_ACCOUNT_TOKEN="glsa_uVd2Na1zl6OhtTSW3Y6aVrEfUYHBODnD_5517eedd"
EOT
```

To compile the bindings for the third party C++ libraries:
```
make bindings
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
