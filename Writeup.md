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

There are two important parts that need to be calibrated in order to get the prediction working for the sensor values from the object (In this case it is a bi cycle). The two are the process noise variables `std_a_` and `std_yawdd_` & the co variance matrix `P_`.

* First let us discuss the linear acceleration noise `std_a_`. Assume the bi cycle can attain a speed of 10 mph in 5 seconds, then its acceleration is going to be 0.8 m/sec^2. I have tried the acceleration between 0.5 to 1 and it worked fine. Details on the RMSE impact on the changes in std_a_ is discussed in section below. Same assumption is made for the `std_yawdd_` and it is also set between 0.5 to 1.0.
* Second variable to initialize is covariance matrix `P_`. It needs to be an identity matrix. I have tried with diagonal values to be 1 to some large numbers.
* Weights for the Sigma points are initialized as given in line # 76 - 84 of ukf.cpp files.
* Set measurement Co varianve vector R for both Laser and Radar as given in line # 100 -110 of ukf.cpp.



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
