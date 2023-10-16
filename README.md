# ObserverDesign

## Install
Recommend using a venv with python 3.8. Run:
```
pip install --upgrade pip
pip install -r requirements.txt
```

To learn more about drake, check out the documentation [here](https://drake.mit.edu/pydrake/index.html)

## Run

### Example 1: Double Integrator

Our first example is a double integrator with torque input and position sensing and a PD controller. The goal is to design an observer that is robust to sensor noise. 

We start with a naive observer that uses the raw sensor signal for position and a finite difference for velocity. You can simulate the resulting system by running:
```
python DoubleIntegrator.py
```

With no noise, it tracks fine, however, if we add 0.01cm std of sensor noise it starts to track poorly:

```
python DoubleIntegrator.py -sensor_noise 0.01
```

#### Example 1.1 (see branch: Example_1_1)
Your goal is to implement the class DIKalmanObserver in DoubleIntegrator.py using a Kalman filter. See [this doc](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf) for a good intro.

As a side goal, take a look through the code to get a feel for how the system is structured in Drake. In particular, look at the methods and properties of the MultiBodyPlant object created by BuildDIPlant. You can use these to calculate many useful values, including jacobians, gravity and cotiolis forces and relative frame positions. For the double integrator we don't need all of this, but it will be more useful as we move onto more complicated examples.

#### Example 1.2 (see branch: Example_1_2)
With the basic filter implemented in Example 1.1, the velocity estimate shows a notable time lag. This in turn triggers oscillations in the velocity tracking of the system

![Example 1.1 Velocity](Figures/Example_1_1_velocity.png)

This is because the dynamics we are using in the filter are not the true dynamics! They are missing the contribution from the actuation torque. The goal for this example is to add the actuation contribution to your filter implementation.

#### Example 1.3 (see branch: Example_1_3)
Adding the actuation input to the model allows us to track the velocity nicely under sensor noise:

![Example 1.2 Velocity](Figures/Example_1_2_velocity.png)

However, our observers don't always have access to the torques applied to the system. If actuation isn't available, a common approach is to use an IMU to measure the accelerations directly. The goal of this example is to update DIKalmanObserverWithAcceleration to include the acceleration measurement from an IMU. Make sure to also include the sensor noise from the IMU in your update!

### Example 2: Cartpole

Our second example is a [cartpole system](http://underactuated.mit.edu/acrobot.html#cart_pole).

![Cartpole diagram](http://underactuated.mit.edu/figures/cartpole.svg)

In our example (CartPole.py), we have set up the system with a position sensor on the pole and an accelerometer on the cart. To control the system, we use a linearization of the system around the [0,pi] position (pole pointed up) to derive a [linear quadratic regulator](http://underactuated.mit.edu/lqr.html) feedback controller. To see the behavior of the controller under perfect state estimation, run:
```
python CartPole.py -cheater_observer True
```

##### Example 2.1 (see branch: Example_2_1)

For this example, we now need to use the position sensor and accelerometer to estimate the 2 positions and velocities of the cartpole system. Your goal is to update CPKalmanObserver to build a Kalman observer linearized around the operating point of the system ([0,pi]). For an example of how to linearize the system, look at CPLQRController. If you are succesful, you should be able to run:
```
python CartPole.py
```
