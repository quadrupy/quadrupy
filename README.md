# ObserverDesign

## Install
Recommend using a venv with python 3.8. Run:
'''
pip install --upgrade pip
pip install -r requirements.txt
'''

## Run

### Example 1: Double Integrator

Our first example is a double integrator with torque input, and position sensing. The goal is to design an observer that is robust to sensor noise. 

We start with a naive observer that uses the raw sensor signal for position and a finite difference for velocity. With no noise, it tracks fine:  

'''
python DoubleIntegrator.py
'''

However, if we add 0.01cm std of sensor noise it starts to track poorly:

'''
python DoubleIntegrator.py -sensor_noise 0.01
'''

Your goal is to implement the class DIKalmanObserver in DoubleIntegrator.py using a Kalman filter. See [this doc](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) for a good intro.
