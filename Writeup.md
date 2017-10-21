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

The NIS values are recorded during the run and plotted as shown in the above pictures. The acceptable values for NIS value should be less than 7.85 (as described in section). The NIS values in both pictures are for most of the time are under 7.85.

### RMSE values

![RMSE values ][image4]
The above picture shows the RMSE values at the end of the time step and the plot of predicted values of the location (in green dots) compared to the Lidar and Radar values from the sensor. Based on the picture the predicted location is along the path and coincides with the sensor measurement.

### What can be done next
* Cleanup the code in Prediction, UpdateLidar/Radar functions.
* Merge common parts of UpdateLidar and UpdateRadar functions and avoid duplicate code.
* Observations using only Lida/ Only Radar measurements during the update of state vector and co-variance matrix.
