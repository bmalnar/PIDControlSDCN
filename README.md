# PIDControlSDCN
PID controller implementation for Udacity Self Driving Car Nanodegree

### Overview
This repository contains implementation of a PID control to steer the car driving on the race track in the Udacity simulator. Essentially, the simulator sends to the PID controller the speed and the steering angle of the vehicle, as well as the value for the cross track error (CTE), i.e. the distance of the car from the center of the track (ideal path). The PID controller processes the data from the simulator, and calculates the desired steering value and the throttle value, which are then sent back to the simulator and used to drive the vehicle. 
The goal of the project is to find the right parameters of the PID controller, so that the vehicle drives around the track successfully, relatively fast and safe, and as close as possible to what humans would consider a comfortable and a safe ride. 

### C++ source files
We have the following C++ source files in the _src_ directory of the repository:

- json.hpp - This file contains all the JSON definitions we need for the project (and more), since JSON is the format used for passing information between our program and the Udacity simulator. 
- main.cpp - This is the main file of the program, which contains the main function. More details on the program flow and the algorithm is given below.  
- PID.cpp and PID.h - These files contain all the implementation code of the PID controller filter. More details on the content is given below. 

### The source files _main.cpp_ and _PID.cpp_

The source files _main.cpp_ and _PID.cpp_ contain the main functions of the program, namely the main function (main.cpp) and all of the steps in the PID controller implementation.

The main function in main.cpp is relatively simple. The important piece to understand how the simulator and the PID controller work together is inside the callback function _onMessage(...)_. With every message, the main program receives the values from the simulator for speed, steering value and CTE. Based on CTE, the PID controller updates the state of the PID controller, namely the proportional, integral and differential errors. This is done in the function _UpdateError(...)_ of the controller. Subsequently, we use the PID controller functions _TotalError()_ and _CalculateThrottle()_ to calculate the values for steering and throttle, respectively. These values are packaged into a JSON message, and sent back to the simulator. The simulator uses these values and sends back the updated values for speed, angle and CTE, and the loop repeats. 

### The parameters of the PID controller

There are overall 5 parameters used in this version of the PID controller. Three of those parameters are used to calculate the steering value based on the proportional, integral and differential errors. The final two parameters are used to calculate a value for throttle, based on the steering value. 

##### Total error calculation

The formula for total error is:

```
total_error = -K_p * p_error - K_i * i_error - K_d * d_error
```

Here, *K_p, K_i, K_d* are coefficients set at the beginning of the program. the values for *p_error, i_error, d_error* are calculated in each simulation step, based on the CTE value received from the simulator. 
- The value of *p_error* is simply set to CTE. In that way, *K_p* and *p_error* attempt to reduce the overal error by the amount proportional to the current cross track error, CTE.  
- The value of *d_error* is the difference between CTE in the current step and CTE in the previous step. In that way, *K_d* and *d_error* attempt to reduce the overal error by the amount proportional to the difference betweeen two consecutive CTE values.   
- The value of *i_error* is a cummulative CTE error summarized over all of the simulation steps. In that way, *K_i* and *i_error* attempt to reduce the overal error by the amount proportional to the sum of all CTE values sees so far. This is useful in the case there is a permanent static component in the CTE value, which cannot be compensated for by *p_error* nor by *d_error*.    

### Choosing the values for *K_p, K_i, K_d*

### Calculating the value for throttle

### How to run the program

After cloning this repository, simply execute the following commands to build and run the program:
```
mkdir build
cd build
cmake ..
make
./pid 0.2 0 5 0.4 0.2
```
At the same time, the Udacity simulator needs to be running, so that the simulator and the PID controller exchange information. 

### Setting up the environment 
- The project is configured to compile with cmake and make. Please make sure that the following dependencies are met:
   - cmake version 3.5
   - make version 4.1 for Linux and Mac and 3.81 for Windows
   - gcc/g++ version 5.4
- Download the Udacity simulator from [here](https://github.com/udacity/self-driving-car-sim/releases/)
- Additional libraries need to be installed by running:
   - On Ubuntu, install-ubuntu.sh 
   - On Mac, install-mac.sh
   - On Windows, the recommended way is to run a virtual machine and use the install-ubuntu.sh script
   
### More information
For even more information on the project structure, dependencies etc. please check original Udacity project [repository](https://github.com/udacity/CarND-PID-Control-Project)
