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

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).


	num_particles = 100;

	double std_x, std_y, std_theta; // Standard deviations for x, y, and psi

	// TODO: Set standard deviations for x, y, and psi.
	 std_x = std[0];
	 std_y = std[1];
	 std_theta=std[2];

	// This line creates a normal (Gaussian) distribution for x.
	normal_distribution<double> dist_x(x, std_x);
	normal_distribution<double> dist_y(y, std_y);
	normal_distribution<double> dist_theta(theta, std_theta);


	for (int i = 0; i < num_particles; ++i) {

		double sample_x, sample_y, sample_theta,sample_weight;
		Particle sample_particle;

		// TODO: Sample  and from these normal distrubtions like this:
		// sample_x = dist_x(gen);
		// where "gen" is the random engine initialized earlier (line 18).

		sample_x = dist_x(gen);
		sample_y = dist_y(gen);
		sample_theta = dist_theta(gen);
		sample_particle.id = i+1;
		sample_particle.x = sample_x;
		sample_particle.y = sample_y;
		sample_particle.theta  = sample_theta;
		sample_particle.weight = 1.0;

		particles.push_back(sample_particle);

		// Print your samples to the terminal.
		//cout << "Sample " << i + 1 << " " << sample_x << " " << sample_y << " " << sample_theta << endl;
	}

	is_initialized = true;


}//end of init function

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and
	// std::default_random_engine useful.
	// http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	// http://www.cplusplus.com/reference/random/default_random_engine/

	double x_init,y_init,theta_init;
	double x_pred,y_pred,theta_pred;
	double std_x,std_y,std_theta;
	std_x = std_pos[0];
	std_y = std_pos[1];
	std_theta = std_pos[2];

	//normal_distribution<double> dist_x(0, std_x);
	//normal_distribution<double> dist_y(0, std_y);
	//normal_distribution<double> dist_theta(0, std_theta);


	for (int i = 0; i < num_particles; ++i) {

		x_init = particles[i].x;
		y_init = particles[i].y;
		theta_init =particles[i].theta;

		  if (fabs(yaw_rate) < 0.00001) {
		      x_pred = x_init + velocity * delta_t * cos(particles[i].theta);
		      y_pred = y_init + velocity * delta_t * sin(particles[i].theta);
		      theta_pred = theta_init;
  	      }else{

  	    	  x_pred = x_init + (velocity/yaw_rate)*(sin(theta_init + yaw_rate*delta_t)-sin(theta_init));
  	    	  y_pred = y_init + (velocity/yaw_rate)*(cos(theta_init)- cos(theta_init + yaw_rate*delta_t));
  	    	  theta_pred = theta_init + yaw_rate*delta_t;
		  }

		normal_distribution<double> dist_x(x_pred, std_x);
		normal_distribution<double> dist_y(y_pred, std_y);
		normal_distribution<double> dist_theta(theta_pred, std_theta);

		x_pred = dist_x(gen);
		y_pred = dist_y(gen);
		theta_pred = dist_theta(gen);

		//x_pred = x_pred + dist_x(gen);
		//y_pred = y_pred + dist_y(gen);
		//theta_pred = theta_pred + dist_theta(gen);

		particles[i].x = x_pred;
		particles[i].y = y_pred;
		particles[i].theta = theta_pred;

		// Print your samples to the terminal.
		//cout << "Init " << i + 1 << " " << x_init << " " << y_init << " " << theta_init << endl;
		//cout << "Predicted " << i + 1 << " " << x_pred << " " << y_pred << " " << theta_pred << endl;
	}

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations){//,std::vector<LandmarkObs_MapId>& data_map ) {
	// TODO: Find the predicted measurement that is closest
	//to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code.
	//But you will probably find it useful to
	//   implement this method and use it as a helper during the
	//updateWeights phase.
	for (int i = 0; i < observations.size(); i++) {

		LandmarkObs o = observations[i];

		//cout << "dataAssociate i: " << o.id << std::endl;
	    // init minimum distance
	    double min_dist = INFINITY;

	    // storing min distance map_id
	    int map_id = -1;

	    for (int j = 0; j < predicted.size(); j++) {

	      LandmarkObs p = predicted[j];

	      //get distance
	      double cur_dist = dist(o.x, o.y, p.x, p.y);

	      // find the predicted landmark nearest the current observed landmark
	      if (cur_dist < min_dist) {
	        min_dist = cur_dist;
	        map_id = p.id;
	      }

	    } //end of j loop

	    //store results of map cordinate observations with closes map_id;
	    observations[i].id = map_id;
	    //LandmarkObs_MapId out_obs_map;
	    //out_obs_map.id = observations[i].id;
	    //out_obs_map.x = observations[i].x;
	    //out_obs_map.y = observations[i].y;
	    //out_obs_map.map_id = map_id;
	    //data_map.push_back(out_obs_map);
	    //cout << "data_map[i].id: "<< data_map[i].id << std::endl;
	    //cout << "data_map[i].map_id: "<< data_map[i].map_id << std::endl;
	  }//end of i loop


}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a
	//       mult-variate Gaussian distribution. You can read
	//       more about this distribution here:
	//       https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE:The observations are given in the VEHICLE'S coordinate system.
	//		Your particles are located according to the MAP'S coordinate system.
	//		You will need to transform between the two systems.
	//   	Keep in mind that this transformation requires both rotation
	//		AND translation (but no scaling).
	//   	The following is a good resource for the theory:
	//   	https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   	and the following is a good resource for the actual equation to
	//		implement (look at equation
	//   	3.33 http://planning.cs.uiuc.edu/node99.html

	 for (int i = 0; i < num_particles; i++) {
	    // get the particle x, y coordinates
	    double p_x = particles[i].x;
	    double p_y = particles[i].y;
	    double p_theta = particles[i].theta;

	    vector<LandmarkObs> predicted;
	    // for each map landmark...
	    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {
	      // get id and x,y coordinates
	      float map_lm_x = map_landmarks.landmark_list[j].x_f;
	      float map_lm_y = map_landmarks.landmark_list[j].y_f;
	      int map_lm_id = map_landmarks.landmark_list[j].id_i;

		  // only consider landmarks within sensor range of the particle
	      double dx = map_lm_x - p_x;
	      double dy = map_lm_y - p_y;
	      if(dx*dx + dy*dy <= sensor_range*sensor_range){
	    	  predicted.push_back(LandmarkObs{ map_lm_id, map_lm_x, map_lm_y });
	      }
	    }//end of predicted loop

		std::vector<LandmarkObs> map_coord_observations;
		for (int j = 0; j < observations.size(); j++) {
			  double x = cos(p_theta)*observations[j].x - sin(p_theta)*observations[j].y + p_x;
			  double y = sin(p_theta)*observations[j].x + cos(p_theta)*observations[j].y + p_y;
			  map_coord_observations.push_back(LandmarkObs{ observations[j].id, x, y });
		 }

		//std::vector<LandmarkObs_MapId> data_map;
		dataAssociation(predicted,map_coord_observations);//,data_map);

		particles[i].weight = 1.0;

		for (int j = 0; j < map_coord_observations.size(); j++) {

		  // placeholders for observation and associated prediction coordinates
		  double observed_x, observed_y;
		  double predicted_x, predicted_y;

		  observed_x = map_coord_observations[j].x;
		  observed_y = map_coord_observations[j].y;
		  int associated_prediction = map_coord_observations[j].id;//.map_id;

		  // get the x,y coordinates of the prediction associated
		  //with the current observation

		  for (int k = 0; k < predicted.size(); k++) {
			if (predicted[k].id == associated_prediction) {
			  predicted_x = predicted[k].x;
			  predicted_y = predicted[k].y;
			}
		  }

		  // calculate weight for this observation with multivariate Gaussian
		  double std_x = std_landmark[0];
		  double std_y = std_landmark[1];
		  double denom = ( 1/(2*M_PI*std_x*std_y));
		  double v_x = pow(predicted_x-observed_x,2)/(2*pow(std_x, 2));
		  double v_y = pow(predicted_y-observed_y,2)/(2*pow(std_y, 2));
		  double obs_w = denom * exp(-(v_x + v_y));

		  //cout << "obs_w: " << obs_w << std::endl;
		  //total weight for particle
		  particles[i].weight *= obs_w;

		}

	    //cout << "i[" << i <<"] particle weight: " << particles[i].weight << std::endl;
	}//end of particles loop

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	vector<Particle> sample_particles;

	  // get max weight
     vector<double> weights;
	 for (int i = 0; i < num_particles; i++) {
	   weights.push_back(particles[i].weight);
	 }
	 double max_weight = *max_element(weights.begin(), weights.end());


	  // generate random starting index for resampling wheel
	  std::discrete_distribution<int> dist(0, num_particles-1);
	  int index = dist(gen);


	  // uniform random distribution [0.0, max_weight)
	  double beta = 0.0;

	  for (int i = 0; i < num_particles; i++) {

	    beta += max_weight * 2.0;
	    while (beta > weights[index]) {
	      beta -= weights[index];
	      index = (index + 1) % num_particles;
	    }
	    sample_particles.push_back(particles[index]);
	  }
	  //Surviving particle will be passed on to next stage.
	  particles = sample_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y)
	//world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
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
