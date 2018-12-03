/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <limits>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	
	if(is_initialized)
	{
		return ;
	}

	default_random_engine gen;

	num_particles = 1000 ;

	Particle p;

	double std_x = std[0] ;
	double std_y = std[1] ;
	double std_theta = std[2] ;

	normal_distribution<double> dist_x(x , std_x) ; 
	normal_distribution<double> dist_y(y , std_y) ;
	normal_distribution<double> dist_theta(theta , std_theta);

	for(unsigned int i=0 ; i< num_particles ; i++)
	{
		p.x = dist_x(gen) ; 
		p.y = dist_y(gen) ;
		p.theta = dist_theta(gen) ;
		p.weight = 1.0 ;

		particles.push_back(p) ;
	}

	is_initialized = true ;
	

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen ;

	double std_x = std_pos[0] ;
	double std_y = std_pos[1] ;
	double std_theta = std_pos[2] ;

	normal_distribution<double> dist_x(0.0 , std_x) ; 
	normal_distribution<double> dist_y(0.0 , std_y) ;
	normal_distribution<double> dist_theta(0.0 , std_theta);

	for(unsigned int i=0 ; i<num_particles ; i++)
	{
		double theta_i = particles[i].theta ;
		double x_f = particles[i].x + (velocity/yaw_rate)*(sin(theta_i + (yaw_rate*delta_t)) - sin(theta_i)); 
		double y_f = particles[i].y + (velocity/yaw_rate)*(cos(theta_i) - cos(theta_i + (yaw_rate*delta_t))) ;
		double theta_f = theta_i + (yaw_rate*delta_t) ;

		particles[i].x = x_f + dist_x(gen) ;
		particles[i].y = y_f + dist_y(gen) ;
		particles[i].theta = theta_f + dist_theta(gen) ;
	}



}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double distance ;
	double min_distance = std::numeric_limits<double>::infinity();
	unsigned int landmark_id = 0.0 ;

	for(unsigned int i=0 ; i<observations.size() ; i++)
	{
		for(unsigned int j=0 ; j<predicted.size() ; j++)
		{
			distance = dist(observations[i].x,observations[i].y,predicted[j].x,predicted[j].y) ;

			if(distance < min_distance)
			{
				min_distance = distance ;
				landmark_id = predicted[j].id ; 
			}

			
		}

		observations[i].id = landmark_id;
	}

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	//run over each particle

	unsigned int num_observations = observations.size() ;
	
	std::vector<LandmarkObs> transformed_observations(num_observations) ;
	
	std::vector<LandmarkObs> predicted_measuremets ;

	for(unsigned int p=0 ; p<num_particles ; p++)
	{
		// transform observed measurements from vehicle's coordinate system to map's coordinate system
		// with respect to each particle

		for(unsigned int i=0 ; i<num_observations ; i++)
		{
			double observation_x = observations[i].x ;
			double observation_y = observations[i].y ;
			double theta = particles[p].theta ;
			transformed_observations[i].x =particles[p].x+(cos(theta)*observation_x)-(sin(theta)*observation_y);
			transformed_observations[i].y =particles[p].y+(sin(theta)*observation_x)+(cos(theta)*observation_y); 
		}

		// predict measurements to all the map landmarks within sensor range for each particle
		
		predicted_measuremets.clear() ;

		for(unsigned int j=0 ; j<map_landmarks.landmark_list.size() ; j++)
		{
			double distance = dist(particles[p].x,particles[p].y,map_landmarks.landmark_list[j].x_f,
								   map_landmarks.landmark_list[j].y_f) ;

			LandmarkObs landmark ;

			if(distance <= sensor_range)
			{
				landmark.id = map_landmarks.landmark_list[j].id_i ;
				landmark.x  = map_landmarks.landmark_list[j].x_f ;
				landmark.y  = map_landmarks.landmark_list[j].y_f ;
				predicted_measuremets.push_back(landmark) ;
			}
			
		}

		// associate the transformed observations with the nearest landmark on the map

		dataAssociation(predicted_measuremets , transformed_observations ) ;

		particles[p].weight = 1.0 ;

		// caluclate particle weight based on the likelihood of the observed measurements

		for(unsigned int k=0; k<num_observations ; k++)
		{
			

			// find coordinates of the associated landmark position

			int associated_id = transformed_observations[k].id ;
			
			double lm_x = -1 ;     		// nearest landmark's x position 
			double lm_y = -1 ;     		// nearest landmark's y position

			

			for(unsigned int l=0 ; l<predicted_measuremets.size() ; l++)
			{
				if(predicted_measuremets[l].id == associated_id)
				{
					lm_x = predicted_measuremets[l].x ;
					lm_y = predicted_measuremets[l].y ; 
					break;
				}
			}

			// calculate each transformed observation probability using
			// multi-variate gaussian probability density function

			// where
			// lm_x is mu_x  and lm_y is mu_y
			// std_landmark[0] is sigma_x and std_landmark[1] is sigma_y

			double gaussian_normalizer = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]) ;
			double exponent = (pow(transformed_observations[k].x-lm_x,2)/(2*pow(std_landmark[0],2)))+
							  (pow(transformed_observations[k].y-lm_y,2)/(2*pow(std_landmark[1],2))) ;
			
			double measurement_probability = gaussian_normalizer*exp(-exponent) ;

			// combine the probabilities of all measurements by taking their product

			particles[p].weight *= measurement_probability ; 


		}



	}




	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
