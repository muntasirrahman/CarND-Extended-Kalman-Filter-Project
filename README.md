## Extended Kalman Filter Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

The goals of this project is to demonstrate how to utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

---

[//]: # (Image References)

[img_dataset1]: images/dataset1.jpg "Data Set 1"
[img_dataset2]: images/dataset2.jpg "Data Set 2"

### Compiling and Usage

To compile, type following commands:
1. mkdir build
2. cd build
3. cmake ..
4. make

To launch the server:
5. ./ExtendedKF

then followed by launching the Term 2 Simulator.
The Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)


### Accuracy

The output values provided by the simulator are:

| RMSE | Value  | Passing Criteria |
|:----:|:------:|:------:| 
| px   | 0.0964 | 0.11 |
| py   | 0.0853 | 0.11 |
| vx   | 0.4154 | 0.52 |
| vy   | 0.4316 | 0.52 |

![Data Set 1][img_dataset1]

![Data Set 2][img_dataset2]


### Follows the Correct Algorithm

#### Follows the general processing flow

The code implements this requirement as shown in file FusionEKF.cpp within `ProcessMeasurement` method.

#### The Kalman Filter algorithm first predicts then updates

The code implements this as shown in FusionEKF.cpp within `ProcessMeasurement` method:
* to predict at line 151 `ekf_.Predict()`
* update process for Radar data from line 163 to line 167 `ekf_.UpdateEKF()`
* update for Lidar data from line 170 to line 172 `ekf_.Update()`

#### The Kalman Filter can handle radar and lidar measurements

During initialization of class KalmanFilter, state vector is updated according to sensor type. For radar, data conversion from polar to cartesian coordinates, is done between line 83 to line 97 of FusionEKF.cpp.

### Code Efficiency

The code avoids unnecessary repeated calculations:
* In kalman_filter.cpp line 74 to line 82, `kalman_gain` is computed once, to update state vector x_ and state covariance matrix P_ .
* In tools.cpp method `CalculateJacobian()` variables c1, c2, c3, posx_vely, posy_velx are computed once, to be used more than once.

### Other Modification
The mac os x has built-in openssl that might be different version than brew's openssl version. The line to find openssl version might refer to different openssl binary. If the openssl version doesn't exist, the compilation process will fail. I made slight improvement of install-mac.sh to fix this problem.


