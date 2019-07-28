/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
    Edited on: Jul 27, 2019
   Eduardo Ribeiro de Campos
 
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;
using namespace std; // inserting namespace std


void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
	// To check the criteria to define the particle qty. See the README File.
	num_particles = 118;  // TODO: Set the number of particles  
	default_random_engine gen; 
  
	/**  reference code - Udacity SDCND - Term2 - class6.5 lines from 40 to 74. */  
  
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]); 

  
  Particle new_particles;
	for (int i = 0; i < num_particles ; i++){
		new_particles.id = i;
		new_particles.x = dist_x (gen);
		new_particles.y = dist_y (gen);
		new_particles.theta = dist_theta (gen);
		new_particles.weight = 1.0;
		particles.push_back(new_particles);
		}
	is_initialized = true;      
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  //To define the number of particles were used the sensor range measure and GPS measurement uncertainty of the sensors
  // defined in the the main.cpp file, lines 33 and 36. more details available in README file
  

	double minimum_yaw_rate = (2*M_PI)/(360*10); // It will be considered yaw rate = 0 , values less than 0.00174 (0.1 degree).

	normal_distribution<double> dist_x(0, std_pos[0]);
	normal_distribution<double> dist_y(0, std_pos[1]);
	normal_distribution<double> dist_theta(0,std_pos[2]);
  
  
  default_random_engine gen;  
  
	for (int i = 0 ; i < num_particles ; i++){
		//Getting the variables:  
		double initial_x_position = particles[i].x;
		double initial_y_position = particles[i].y;
		double initial_yaw_rate = particles[i].theta;
		double final_x_position;
		double final_y_position;
		double final_yaw; 
      
		if((abs(yaw_rate)) < minimum_yaw_rate){
			final_x_position = initial_x_position + velocity * delta_t * cos(initial_yaw_rate);
			final_y_position = initial_y_position + velocity * delta_t * sin(initial_yaw_rate);
			final_yaw = initial_yaw_rate;
		}
		else{
			final_x_position = initial_x_position + (velocity / yaw_rate )*(sin(initial_yaw_rate + yaw_rate * delta_t)-sin(yaw_rate));
			final_y_position = initial_y_position + (velocity / yaw_rate )*(cos(yaw_rate)-cos(initial_yaw_rate + yaw_rate * delta_t));
			final_yaw = initial_yaw_rate + yaw_rate * delta_t;      
		}  
		// Updating values.
		particles[i].x = final_x_position ;
		particles[i].y = final_y_position ;
		particles[i].theta = final_yaw;
     
		// Adding noise.
		particles[i].x += dist_x (gen);
		particles[i].y += dist_y (gen);
		particles[i].theta += dist_theta (gen);
	}
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
   // Declaring the start minimal distance as biggest as possible
   // according to the map_data the biggest distance possible could be found with the hypotenuse from the grid limits
   // X = 350 , Y = 175 picture available in the README file  
    
	double landmark_min_distance = sqrt(pow(350,2) + pow(175,2));
  
	for (float i = 0 ; i < observations.size(); i++){
    //for(vector<float>::size_type i = 0; i < observations.size(); i++){
		float x_coord_observed = observations[i].x;
		float y_coord_observed = observations[i].y;

		for (float j = 0 ; j < predicted.size(); j++){
		//for(vector<float>::size_type j = 0; i < predicted.size(); j++){
			double min_distance;
			float x_coord_predicted = predicted[j].x;
			float y_coord_predicted = predicted[j].y;
			int id_predicted =  predicted[j].id;         
			// From helper_functions.h line 37. inline double dist(double x1, double y1, double x2, double y2)
			min_distance = dist (x_coord_observed,y_coord_observed,x_coord_predicted,y_coord_predicted);
			if (min_distance < landmark_min_distance){
				landmark_min_distance = min_distance;
				observations[i].id = id_predicted;                   
			}                
		}    
	}
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  /**
   Acc. to class 6.15, it will be necessary execute the followings Steps for each particle:
  1. Transform
  2. Associate
  3. Update weights:
     3.1 Determine measurement probabilities.
     3.2 Combine probabilities
  */  
  
	for (int i = 0; i < num_particles; i++) {  
		double x_particle = particles[i].x;
		double y_particle = particles[i].y;
		double theta_particle = particles[i].theta;      

		//1.1 - Converting the observations from car to map coordinate system 
		vector<LandmarkObs> observations_transformed;
			for (float j = 0 ; j < observations.size(); j++){
      		//for(vector<float>::size_type j = 0; j < observations.size(); j++){
			float x_observed_car_coord = observations[j].x;
			float y_observed_car_coord = observations[j].y;      
     
			LandmarkObs transformed_info = {};
			transformed_info.x = x_particle + cos(theta_particle)*x_observed_car_coord - sin(theta_particle)*y_observed_car_coord ;        
			transformed_info.y = y_particle + sin(theta_particle)*x_observed_car_coord + cos(theta_particle)*y_observed_car_coord ;         
			transformed_info.id = observations[j].id;
          
			observations_transformed.push_back(transformed_info);
        }
		//2.1 Filtering the landmarks inside the sensor range from the car using informations from map_data file:
		vector<LandmarkObs> Landmarks_in_car_range; 
      
		// defining the maximum sensor range ( sensor range + standard deviation).
		float sensor_max_distance = sensor_range + std_landmark[0];      
		for (float j = 0 ; j < map_landmarks.landmark_list.size(); j++){          
		//for(vector<float>::size_type j = 0; j < map_landmarks.landmark_list.size(); j++){
			//  See more informations regard Map_data in helper_functions.h file lines: from 85 to 122.
			float map_landmark_x_coord = map_landmarks.landmark_list[j].x_f;
			float map_landmark_y_coord = map_landmarks.landmark_list[j].y_f;
			int map_landmark_id = map_landmarks.landmark_list[j].id_i; 
          
			// From helper_functions.h line 37. inline double dist(double x1, double y1, double x2, double y2)
			double landmark_dist = dist (x_particle,y_particle,map_landmark_x_coord,map_landmark_y_coord);
			if (landmark_dist < sensor_max_distance){
				LandmarkObs info_landmarkes = {};
				info_landmarkes.id = map_landmark_id;
				info_landmarkes.x = map_landmark_x_coord;
				info_landmarkes.y = map_landmark_y_coord;
				Landmarks_in_car_range.push_back(info_landmarkes);
			}          
		} 
  
		//2.2 Associating the nearest landmark to get x and y coordinates.
		dataAssociation(Landmarks_in_car_range,observations_transformed );  
      
		// 3.1 Determine measurement probabilities.  
		double weight_particle = 1.0;
        for (float j = 0 ; j < Landmarks_in_car_range.size(); j++){  
		//for(vector<float>::size_type j = 0; j < Landmarks_in_car_range.size(); j++){
			double mu_x = Landmarks_in_car_range[j].x;
			double mu_y = Landmarks_in_car_range[j].y;  
			int id_landmark = Landmarks_in_car_range[j].id;           
			for (float k = 0 ; k < observations_transformed.size(); k++){ 
          	//for (vector<float>::size_type k = 0; k < observations_transformed.size(); k++){          
 
				int id_observed = observations_transformed[j].id; 

				if (id_landmark == id_observed){
					double x_observed_coord = observations_transformed[k].x;
					double y_observed_coord = observations_transformed[k].y;           
					// calculate normalization term          
					double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);  
					// calculate exponent  
					double exponent = (pow(x_observed_coord - mu_x, 2) / (2 * pow(std_landmark[0], 2)))
                            + (pow(y_observed_coord - mu_y, 2) / (2 * pow(std_landmark[1], 2)));
                  
					// calculate weight using normalization terms and exponent                  
					double individual_landmark_weight = gauss_norm * exp(-exponent);
					if ( individual_landmark_weight > 0){                 
						weight_particle *= individual_landmark_weight;
					}                      
					else{                     
						weight_particle *= 0.00000001;                       
					}                  
				}                  
			}              
		particles[i].weight = weight_particle;
		}              
	}
}  
  
void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

	for ( int i = 0 ; i < num_particles ;i++){ 
      weights.push_back(particles[i].weight);
     }
  
   double partial_value;
   for (size_t i = 0; i < weights.size();i++){
     double ind_weight = weights[i];
     partial_value += ind_weight;
    }
    double sum_weights = partial_value;
	for ( int i = 0 ; i < num_particles;i++){ 
      particles[i].weight /= sum_weights ;
     }

  // clear the weight vector
	weights.clear();  

}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}