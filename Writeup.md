## Unscented Kalman Filter Project

The goals / steps of this project are the following:
* Initialize Unscented Kalman filter variables and in particular modify the process noise variables (`std_a_`, `std_yawdd_`) and  Covariance vector
 (`P_`) for the sensor.
* Implement Prediction function that predicts mean values of data points (`x_`) and state co-variance matrix (`P_`)
* Implement `Predction` and `UpdateLidar/Radar` functions.
* Dump the NIS values at each timestep and see if they are within 95 percentile most of the time.
* Make sure the RMSE values for pX, pY, vX and vY are less than .09, .10, .40, .30 respectively.


[//]: # (Image References)

[image1]: ./images/lidar_nis_plot.png "NIS values for Lidar Data"
[image2]: ./images/radar_nis_plot.png "NIS values for Radar Data"
[image3]: ./images/rmse_P_noise.png "RMSE values based on Covariance & Process variables"
[image4]: ./images/rmse_in_simulator.png "RMSE values"
[image5]: ./images/lidar_only_nis_plot.png "NIS values with Lidar Only Data"
[image6]: ./images/radar_only_nis_plot.png "NIS values with  Radar Only Data"
[image7]: ./images/rmse-in_simulator_inlidar_only.png "RMSE values only with Lidar data"
[image8]: ./images/rmse-in_simulator_inradar_only.png "RMSE values only with Radar data"
[image9]: ./images/rmse_in_simulator_ekf.png "RMSE values for EKF"

### Initialization of UKF variables

There are two important parts that need to be calibrated in order to get the prediction working for the sensor values from the object (In this case it is a bi cycle). The two are the process noise variables `std_a_` and `std_yawdd_` & the co variance matrix `P_`.

* First let us discuss the linear acceleration noise `std_a_`. Assume the bi cycle can attain a speed of 10 mph in 5 seconds, then its acceleration is going to be 0.8 m/sec^2. I have tried the acceleration between 0.5 to 1 and it worked fine. Details on the RMSE impact on the changes in std_a_ is discussed in section below. Same assumption is made for the `std_yawdd_` and it is also set between 0.5 to 1.0.
* Second variable to initialize is covariance matrix `P_`. It needs to be an identity matrix. I have tried with diagonal values to be 1 to some large numbers.
* Weights for the Sigma points are initialized as given in line # 76 - 84 of ukf.cpp files.
* Set measurement Co varianve vector R for both Laser and Radar as given in line # 100 -110 of ukf.cpp.



### Implementation of Prediction function
This function predicts the state vector and co-variance matrix with reference to the data from the last time step. It essentially predicts `x_` and `P_` based on the info from the last known information.

The key here is to predict the sigma points and derive state vector from the sigma points and the weights (line # 300 - 350 of ukf.cpp). Similarly use the predicted sigma points data to generate co-variance matrix `P_` (as shown in line # 350 - 368 of ukf.cpp)

### Implementation of UpdateLidar/Radar function
The algorithm is relatively is same for both the Lidar and Radar. The only difference is the input data, and the measurement co-variance matrix (`R_Laser`/`R_Radar`).

* As a first step, convert the sigma points identifed in prediction step to the measurement space.
* Find the difference between predicted sigma points and the state vector. Similarly find the difference between predicted values vs measurement space sigma points to generate Cross correlation matrix (`Tc`) .
* From `Tc` find Kalman Gain MAtrix (`K`)
* From `K` update the state vector `x_` and co variance vector `P_`.
* Also calculate the NIS values (line#461 for Lasar update) and log them into a file to review the values for the whole session.


### Effect of Covariance and Process variables on RMSE values

![RMSE values based on Covariance & Process variables][image3]

The above table shows the RMSE values for a given set of process noise variables (`std_a_`, `std_yawdd_`) and Co-variance values (`P_`).

The initial values of process noise variables are given as 30, 30 which is very large for a bicycle. I've tried to come up with a number that is reasonable for a bicycle based on the acceleration it can attain in 5 seconds. As described in Initialization section, the acceptable range is between 0.5 to 1.0. I've tried with varying range of values for the noise variables and found that RMSE values are in acceptable range as long as the process noise variables are less than 1.0 no matter what the `P_` values are (as long as the diagonal values are set).

### NIS values for Lidar and Radar

![NIS values for Lidar Data][image1]

![NIS values for Radar Data][image2]

The NIS values are recorded during the run and plotted as shown in the above pictures. The acceptable values for NIS value should be less than `2*stdev` (as described in section).
* For Radar NIS values the 95 percentile values around 4.5 and as you can see in the picture most of the values are under 4.5.

* For Lidar NIS values the 95 percentile values around 3.53 and as you can see in the picture most of the values are under 3.53.

![NIS values for Lidar Only Data][image5]
The above diagram shows the NIS data for Lidar data only when the Lidar sensor data is used for the Measurement update for both state vector (`x_`) and co-variance matrix (`P_`). Here the 95 percentile data for NIS value is about 4.65 which is higher  than (4.5) the 95 percentile data when both Lidar and Radar sensor data is used.

![NIS values for Radar Only Data][image6]
The above diagram shows the NIS data for Radar data only when the Radar sensor data is used for the Measurement update for both state vector (`x_`) and co-variance matrix (`P_`). Here the 95 percentile data for NIS value is about 3.62 which is higher  than (3.53) the 95 percentile data when both Lidar and Radar sensor data is used.

By looking at NIS values when only one of the sensor is used or both sensor data is used, it shows that using both sensor data is beneficial when it comes to predicting the next position of the tracking vehicle. I've aslo observed the RMSE values when individual sensor data is used.

###### RMSE values for Lidar Only data
![RMSE values for Lidar Only data ][image7]
The above picture shows the RMSE values only with the Lidar data. In this case the RMSE values for x and y are close to values when both sensors are used. This is due to the fact that the Laser measurements has only x and y data and so the Prediction for x and y are close to ground truth.

###### RMSE values for Lidar Only data
![RMSE values for Radar Only data ][image8]
The above picture shows the RMSE values only with the Radar data. In this case the RMSE values for vx and vy are close to values when both sensors are used. This is due to the fact that radar ground truth data get data polar co-ordinates and it is easy to calculate velocity using the radial velocity measurement.

### RMSE values with both sensors using UKF

![RMSE values ][image4]
The above picture shows the RMSE values at the end of the time step and the plot of predicted values of the location (in green dots) compared to the Lidar and Radar values from the sensor. Based on the picture the predicted location is along the path and coincides with the sensor measurement.

### RMSE values with both sensors using EKF
![RMSE values using EKF ][image9]
The above picture shows the RMSE values using EKF. The RMSE values using EKF are lesser due to the fact that the prected values in EKF uses sigma points for both Lidar and Radar data to predict and update `x_` and `P_`.

### House keeping things to take care
* Cleanup the code in Prediction, UpdateLidar/Radar functions.
* Merge common parts of UpdateLidar and UpdateRadar functions and avoid duplicate code.

### Next steps
* Try the Bonus challenge to catch the run away car UKF.
