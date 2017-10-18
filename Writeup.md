## Unscented Kalman Filter Project

The goals / steps of this project are the following:
* Initialize Unscented Kalman filter variables and in particular modify the process noise variables (`std_a_`, `std_yawdd_`) and  Covariance vector
 (`P_`) for the sensor.
* Implement Prediction function that predicts mean values of data points (`x_`) and state co-variance matrix (`P_`)
* Implement `Predction` and `UpdateLidar/Radar` functions.
* Dump the NIS values at each timestep and see if they are within 7.85 most of the time.
* Make sure the RMSE values for pX, pY, vX and vY are less than .09, .10, .40, .30 respectively.


[//]: # (Image References)

[image1]: ./images/lidar_nis_plot.png "NIS values for Lidar Data"
[image2]: ./images/radar_nis_plot.png "NIS values for Radar Data"
[image3]: ./images/rmse_P_noise.png "RMSE values based on Covariance & Process variables"
[image4]: ./images/rmse_in_simulator.png "RMSE values"

### Initialization of UKF variables

### Implementation of Prediction function

### Implementation of UpdateLidar/Radar function

### Effect of Covariance and Process variables on RMSE values

![RMSE values based on Covariance & Process variables][image3]

### NIS values for Lidar and Radar

![NIS values for Lidar Data][image1]

![NIS values for Radar Data][image2]

### RMSE values

![RMSE values ][image4]

### Conclusion
