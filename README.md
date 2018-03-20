# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project, Extended kalman filter is implemented  in C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
* On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Overview

The three main steps for programming a Kalman filter:
1. initializing Kalman filter variables.
2. predicting where our object is going to be after a time step \Delta{t}Δt
3. updating where our object is based on sensor measurements

Then the prediction and update steps repeat themselves in a loop.

Implementation:
1. main.cpp uses uWebSocketIO in communicating with the simulator.
2. FusionEKF.cpp/FusionEKF.h contains implementation of initialising Kalman Filter variables and Function call to predict and update KF
3. kalman_filter.cpp/kalman_filter.h contains implementaion of Kalman Filter Predict and Update functions for Laser and Radar Sensor measurements.
4. tools.cpp/tools.h contains  1.Jacobian Matrix implementation for Extended Kalman Filter used in Radar sensor measurements and 2. Calculation of RMSE based on the calculated estimates and ground truth values of object position and velocity in X-Y cordinates.

## Results
For Dataset 1, following are the rmse values:
X: 0.0961
Y: 0.0846
VX: 0.4484
VY: 0.4330



