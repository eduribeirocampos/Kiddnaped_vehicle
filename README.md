[//]: # (Image References)
 
[image1]: ./Support/Map_data_table.jpg 
[image2]: ./Support/map_data_visualization.jpg
[image3]: ./Support/Num_Particles_def.jpg
[image4]: ./Support/Particles_creation.jpg
[image5]: ./Support/Prediction_Equations.jpg 
[image6]: ./Support/yaw_rate_precision.jpg 
[image7]: ./Support/Data_association_function.jpg
[image8]: ./Support/transform_equation.jpg
[image9]: ./Support/Land_marks_inside_sensor_range.jpg 
[image10]: ./Support/Nearest_landmark_association.jpg
[image11]: ./Support/particle_weight_formula.jpg
[image12]: ./Support/particles_weight.jpg
[image13]: ./Support/total_weight_formula.jpg
[image14]: ./Support/Normalization_formula.jpg
[image15]: ./Support/Simulator_interface.jpg
[image16]: ./Support/success.jpg



# **Kidnapped Vehicle Project** 
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


# Overview
This repository contains all the code needed to complete the final project for the Localization course in Udacity's Self-Driving Car Nanodegree.



## Project Introduction
Your robot has been kidnapped and transported to a new location! Luckily it has a map of this location, a (noisy) GPS estimate of its initial location, and lots of (noisy) sensor and control data.

In this project you will implement a 2 dimensional particle filter in C++. Your particle filter will be given a map and some initial localization information (analogous to what a GPS would provide). At each time step your filter will also get observation and control data.

All the input files was provided by udacity in  [Kidnapped Vehicle project Github repository](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project)

To conclude the project is necessary reach the goals according to the project [rubric](https://review.udacity.com/#!/rubrics/747/view).

To run the program is possible in Udacity workspace provided in the class environment. To run locally there are some instruction and guidelines to follow, more details available in this [ORIGINAL UDACITY_README](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project/blob/master/README.md) file.



# Starting the Project.

## The main tasks:

The only file necessary to modify is particle_filter.cpp in the src directory. The file contains the scaffolding of a ParticleFilter class and some associated methods. Read through the code, the comments, and the header file particle_filter.h to get a sense for what this code is expected to do.

## Inputs to the Particle Filter:

You can find the inputs to the particle filter in the data directory.

The Map*
map_data.txt includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns

- x position<br/>
- y position<br/> 
- landmark id<br/> 

Below is possible see the first 5 rows from [map_data.txt](./data/map_data.txt):

![alt text][image1]

In the next picture we have the data in the 2D cartesian chart: 
![alt text][image2]

The visualization above from [map_data.txt](./data/map_data.txt) was creating using a `Jupyter notebook` file, named [Load_map.ipynb](./Support/Load_map.ipynb).

# Exploring the particle_filter.cpp file:

The main objective here is complete the functions from Particle_filter class , as shown in next list:

1. init.<br/>
2. prediction.<br/>
3. dataAssociation.<br/>
4. updateWeights.<br/>
5. resample.<br/>


### 1. init. - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from 29 to 59).

init Initializes particle filter by initializing particles to Gaussian distribution around first position and all the weights to 1.<br/>
@param x Initial x position [m] (simulated estimate from GPS).<br/>
@param y Initial y position [m].<br/>
@param theta Initial orientation. [rad]<br/>
@param std[] Array of dimension 3 [standard deviation of x [m],standard deviation of y [m], standard deviation of yaw [rad]].<br/>

1.1 TODO_1: Set the number of particles. Initialize all particles to first position (based on estimates of x, y, theta and their uncertainties from GPS) and all weights to 1.<br/>
1.2 TODO_2: Add random Gaussian noise to each particle.<br/>

*NOTE: Consult [particle_filter.h](./src/particle_filter.h) for more information about this method (and others in this file).*

To define the number of particles were used the sensor range measure and GPS measurement uncertainty of the sensors  defined in the the [main.cpp](./src/main.cpp) file, lines 33 and 36. 

![alt text][image3]

The next picture shows genericly how the particles will be created adding random Gaussian noise.

![alt text][image4]


### 2. prediction - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from 61 to 113).

prediction Predicts the state for the next time step using the process model.
@param delta_t Time between time step t and t+1 in measurements [s].<br/>
@param std_pos[] Array of dimension 3 [standard deviation of x [m],standard deviation of y [m], standard deviation of yaw [rad]].<br/>
@param velocity Velocity of car from t to t+1 [m/s].<br/>
@param yaw_rate Yaw rate of car from t to t+1 [rad/s].<br/>

2.1. TODO: Add measurements to each particle and add random Gaussian noise.<br/>

*NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.**
- http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
- http://www.cplusplus.com/reference/random/default_random_engine/*

In this step,it will be considdered the Equations below:

![alt text][image5]

To get a yaw angle precision over than 0.1 degree , it will be considered Yaw rate angle = 0 for absolute values less than 1.74E-3:

![alt text][image6]

### 3. dataAssociation - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from115 to 148).

3.1. TODO: Find the predicted measurement that is closest to each observed measurement and assign the observed measurement to this particular landmark.<br/>
*NOTE: this method will NOT be called by the grading code. But you will probably find it useful to implement this method and use it as a helper during the updateWeights phase.*

The next picture shows as generic example the output result from this function:

![alt text][image7]

### 4. updateWeights - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from150 to 262).

TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read more about this distribution here:<br/>
https://en.wikipedia.org/wiki/Multivariate_normal_distribution<br/>
*NOTE: The observations are given in the VEHICLE'S coordinate system.<br/>
Your particles are located according to the MAP'S coordinate system.<br/>
You will need to transform between the two systems. Keep in mind that this transformation requires both rotation AND translation (but no scaling).<br/>
The following is a good resource for the theory:
https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
and the following is a good resource for the actual equation to implement(look at equation 3.33) http://planning.cs.uiuc.edu/node99.html*

To undertand the function inputs format is necessary check the [main.cpp](./src/main.cpp) file. 

At line 111 we have the function: `pf.updateWeights(sensor_range, sigma_landmark, noisy_observations, map)`.About the variables: 

- **sensor_range:** It is a int value equal 50 (line 33)<br/>
- **sigma_landmark:** It is the Landmark measurement uncertainty [x [m], y [m]] equal to {0.3, 0.3} (line 38)<br/>
- **noisy_observations:** It is the receipted noisy observation data from the simulator.[{obs_x,obs_y},{obs_x,obs_y},...{obs_x,obs_y}]
- **map:** It is the file [map_data.txt](./data/map_data.txt)


Acc. to class 6.15, it will be necessary execute the followings steps for each particle:

1. Transform.<br/>
2. Associate.<br/>
3. Update weights<br/>
    3.1 Determine measurement probabilities.<br/>
    3.2 Combine probabilities.<br/>

#### 4.1-Transforming - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from181 to 191).

Here it will be applied the next equations to convert the Observed landmarks coordinates from the car to map system.

![alt text][image8]

#### 4.2- Filtering the landmarks inside the sensor range - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from193 to 214).

It was consideres as sensor maximum radius as range the sum of sensor range and the standars deviation.

![alt text][image9]

#### 4.3- Associating - [particle_filter.cpp](./src/particle_filter.cpp) file (line 217).

In this Step , it will be checked the Landmarks observed and already transformed in the map coordinates system and compared with the landmarks inside the sensor range , in order to apply the real landmark id in the observed landmark , in this case is used the Nearest criteria.


![alt text][image10]


#### 4.4- Calculating the partial weights for each landmark - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from219 to 249).

In this step is calculate the weight for each landmark inside the sensor range and appended in a vector.

below we have the formula:

![alt text][image11]

The next Picure shows a generic situation:

![alt text][image12]


#### 4.5- Calculating the total weights for each Particle - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from 250 to 262).

The particles final weight will be calculated as the product of each partial weights already calculated in the previous step.

![alt text][image13]

### 5. resample - [particle_filter.cpp](./src/particle_filter.cpp) file (lines from 264 to 290).

5.1. TODO: Resample particles with replacement with probability proportional to their weight.<br/> 
*NOTE: You may find std::discrete_distribution helpful here.<br/>
http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution*

With the individual particle weight  calculated in the previous function. here we will append in a unique vector named `weight`, all the weights in orther to normalize each particle weight. see the formula below:

![alt text][image14]

As second step, using the function `std::discrete_distribution` and `std::default_random_engine` we replace the particles vector taking account the highest probability of the each particle.

# Reaching the goals !!

Using the wokspace available in the class enviroment with support a GPU and a link to the simulator:

![alt text][image15]


We simulate the algorithm and receive a message if all the requirements are OK.

**click on the picture to see an example video**

[![alt text][image16]](https://youtu.be/HLA7H9usUyE)
