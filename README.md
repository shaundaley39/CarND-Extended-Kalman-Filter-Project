# Extended Kalman Filter Project - Object Tracking

Road vehicles do not move through a static environment. Driving in traffic requires reasoning about many moving objects - notably other vehicles, but also cyclists, pedestrians, animals and countless (perhaps less common) moving things. This project explores the use of a Kalman filter in solving this problem.

# (Extended) Kalman Filters

A Kalman filter is an approximation of a Bayesian filter, making a number of assumptions to track the state of an object:
- the object's state can be reasonably represented by a unimodal Gaussian approximation
- we arrive at a posterior estimate, given only a prior estimata and a single new measurement. If we are given new information about a past state of the object (e.g. measurement packets arriving out-of-order), that information is neglected (even if it's only a millisecond late). This assumption means we don't need to persist a history of prior states, or use prior states in computation - this vastly reduces the computational cost of a Kalman filter, and is a reasonable trade-off (it's important that our sensors are well synchronized and packets usually aren't arriving out-of-order!).
- a Kalman filter assumes that a linear approximation of motion is adequate, and can be implemented with a matrix multiplication. An extended Kalman filter relaxes this assumption.
- a Kalman filter assumes that measurement updates are linear and can be implemented with matrix multiplications. An extended Kalman filter relaxes this assumption.

A Kalman (or extended Kalman) filter consists of an update loop. Each time a measurement packet is received (with information on the state of the object):
- the timestamp of that new measurement is used to update the object's predicted state (and uncertainty) from the prior time to the current time
- the new measurement is processed to update the object's state (and uncertainty)

The resulting state representation can then be used to approximate the current and near-future state of the object being tracked.

#  This Project Specifically

This project combines a linear motion model (Kalman style) and one linear measurement model (lidar data - Kalman style) with a non-linear measurement model (radar - extended Kalman style). 

The simulator provides a visualization: our Kalman filter state tracks the object's actual trajectory closely, even as the object turns abruptly (breaking linear motion assumptions) and despite noisy sensor data.

Using the RMSE to compare the Kalman filter state with a groundtruth, we obtain nice results with both of the datasets included in this repository (see below for more information). On the first data set, for x, y, v_x and v_y respectively, we have RMSE values better than [.11, .11, 0.52, 0.52].

# Limitations and Some Reflections

This Kalman filter implementation is coded specifically for the situation at hand - a single filter being updated based on state estimates from one lidar sensor and one radar sensor. A better implementation would attempt to be extensible/ generic, and would provide better support for quick many-sensor fusion.

It would be fruitful to explore the integration of a Kalman filter into a larger system: perhaps an event-based pipeline? With inference of object path plans (e.g. with probability P, vehicle A is shifting from the right-most to the centre lane)?

Before getting too deep into either (A) developing a full-blown Kalman filter library or (B) exploring the integration of Kalman filters in useful systems, it would be worth exploring unscented Kalman filters.

---

## Build and Run

To try out this Kalman filter, it's necessary to use the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Generating Additional Data

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.
