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

#define EPS 0.001

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

	num_particles = 50 ;

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
		p.id = i ;

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


	double x_f;
	double y_f;
	double theta_f;

	for(unsigned int i=0 ; i<num_particles ; i++)
	{	
		double theta_i = particles[i].theta ;

		if(fabs(yaw_rate) < EPS)
		{
			x_f = particles[i].x + (velocity*delta_t*cos(theta_i));
			y_f = particles[i].y + (velocity*delta_t*sin(theta_i));
			theta_f = theta_i;
		}
		
		else
		{
			
			x_f = particles[i].x + (velocity/yaw_rate)*(sin(theta_i + (yaw_rate*delta_t)) - sin(theta_i)); 
			y_f = particles[i].y + (velocity/yaw_rate)*(cos(theta_i) - cos(theta_i + (yaw_rate*delta_t)));
			theta_f = theta_i + (yaw_rate*delta_t) ;
		}

		// add gaussian noise 

		particles[i].x = x_f + dist_x(gen) ;
		particles[i].y = y_f + dist_y(gen) ;
		particles[i].theta = theta_f + dist_theta(gen) ;
	}



}

std::vector<LandmarkObs> ParticleFilter::transform_observations(const std::vector<LandmarkObs>& observations , const Particle & particle)
{
	unsigned int num_observations = observations.size() ;

	std::vector<LandmarkObs> transformed_observations(num_observations) ; 

	for(unsigned int i=0 ; i<num_observations ; i++)
	{
		double observation_x = observations[i].x ;
		double observation_y = observations[i].y ;
		double theta = particle.theta ;
		transformed_observations[i].x =particle.x+(cos(theta)*observation_x)-(sin(theta)*observation_y);
		transformed_observations[i].y =particle.y+(sin(theta)*observation_x)+(cos(theta)*observation_y); 
	}

	//cout<<"transformation done"<<endl;

	return transformed_observations ;

}

std::vector<LandmarkObs> ParticleFilter::find_inRange_landmarks(const Particle &particle , const Map &map_landmarks , const double sensor_range)
{
	std::vector<LandmarkObs> inRange_landmarks ;
	
	for(unsigned int j=0 ; j<map_landmarks.landmark_list.size() ; j++)
	{
		double distance = dist(particle.x,particle.y,map_landmarks.landmark_list[j].x_f,map_landmarks.landmark_list[j].y_f);
										
		LandmarkObs landmark ;

		if(distance <= sensor_range)
		{	
			landmark.id = map_landmarks.landmark_list[j].id_i ;
			landmark.x  = map_landmarks.landmark_list[j].x_f ;
			landmark.y  = map_landmarks.landmark_list[j].y_f ;
			inRange_landmarks.push_back(landmark) ;
		}
		
	}
	
	/*
	if(inRange_landmarks.size() != 0 )
	{
		cout<<"inRange landmarks found"<<endl;
	}
	else
	{
		cout<<"inRange landmarks not found !!"<<endl;
	}
	cout<<"\ninRange landmarks:";
	for(unsigned int h=0;h<inRange_landmarks.size() ; h++)
	{
		cout<<inRange_landmarks[h].id<<","<<inRange_landmarks[h].x<<","<<inRange_landmarks[h].y<<endl;
	}
	*/
	return inRange_landmarks ;

}

std::vector<LandmarkObs> ParticleFilter::dataAssociation(const std::vector<LandmarkObs>& observations ,const  std::vector<LandmarkObs>& inRange_landmarks) 
{
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	double distance ;
	double min_distance ;
	

	LandmarkObs nearest_landmark;
	std::vector<LandmarkObs> associated_landmarks ;
	

	for(unsigned int i=0 ; i<observations.size() ; i++)
	{
		// nearest neighbour association

		for(unsigned int j=0 ; j<inRange_landmarks.size() ; j++)
		{
			distance = dist(observations[i].x,observations[i].y,inRange_landmarks[j].x,inRange_landmarks[j].y) ;

			if(j==0)
			{
				min_distance = distance ;
				nearest_landmark = inRange_landmarks[0] ;
			}
			

			if(distance < min_distance)
			{
				min_distance = distance ;
				nearest_landmark = inRange_landmarks[j] ; 
			}

			
		}

		
		associated_landmarks.push_back(nearest_landmark) ;
	}

	/*
	if(associated_landmarks.size() != 0)
	{
		cout<<"data association done"<<endl;
	}
	else
	{
		cout<<"data association not done"<<endl;
	}
	*/
	return associated_landmarks ;

}

double ParticleFilter::calculate_weight(const std::vector<LandmarkObs>& transformed_observations , const std::vector<LandmarkObs>& associated_landmarks , double std_landmark[])
{
	double weight = 1.0;
	
	unsigned int num_observations = transformed_observations.size() ;

	for(unsigned int k=0; k<num_observations ; k++)
	{
	

		// get map coordinates of the associated landmark position
		
		double lm_x = associated_landmarks[k].x ;     		// nearest landmark's x position 
		double lm_y = associated_landmarks[k].y ;     		// nearest landmark's y position
		
		// calculate each transformed observation probability using
		// multi-variate gaussian probability density function
		
		double gaussian_normalizer = 1.0/(2*M_PI*std_landmark[0]*std_landmark[1]) ;
		double exponent = (pow(transformed_observations[k].x-lm_x,2)/(2*pow(std_landmark[0],2)))+
						  (pow(transformed_observations[k].y-lm_y,2)/(2*pow(std_landmark[1],2))) ;
			
		double measurement_probability = gaussian_normalizer*exp(-exponent) ;

		// combine the probabilities of all measurements by taking their product

	    weight *= measurement_probability ; 

	}

	return weight;

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

	//initialization

	unsigned int num_observations = observations.size() ;
	
	std::vector<LandmarkObs> transformed_observations(num_observations) ;

	std::vector<LandmarkObs> associated_landmarks(num_observations) ;

	std::vector<LandmarkObs> inRange_landmarks ;

	weights.clear();

	//run over each particle

	for(unsigned int p=0 ; p<num_particles ; p++)
	{
		//initialization 

		particles[p].weight = 1.0 ;
		
		// transform observed measurements from vehicle's coordinate system to map's coordinate system
		// with respect to each particle

		transformed_observations = transform_observations(observations , particles[p]);

		// find all the map landmarks within sensor range for each particle
		
		inRange_landmarks = find_inRange_landmarks(particles[p], map_landmarks , sensor_range) ;

		// associate the transformed observations with the nearest landmark on the map

		
		associated_landmarks = dataAssociation(transformed_observations , inRange_landmarks) ;


		// caluclate particle weight based on the likelihood of the observed measurements
			
		particles[p].weight = calculate_weight(transformed_observations,associated_landmarks,std_landmark);

		

		weights.push_back(particles[p].weight) ;


	}




	

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> new_particles;
	
	default_random_engine gen ;
	
	discrete_distribution<int> resampler(weights.begin() , weights.end() ) ;
	

	for(unsigned int i=0 ; i<num_particles ; i++)
	{
		new_particles.push_back(particles[resampler(gen)]) ;
	}

	particles = new_particles ;


}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();
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
