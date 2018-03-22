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

void ParticleFilter::init(double x, double y, double theta, double gps_std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    num_particles = 500;
    
    default_random_engine gen;
    normal_distribution<double> dist_x(x, gps_std[0]);
    normal_distribution<double> dist_y(y, gps_std[1]);
    normal_distribution<double> dist_theta(theta, gps_std[2]);
    
    for (int i = 0; i < num_particles; i++){
        Particle p = {
            i, // set id to i ?
            dist_x(gen),
            dist_y(gen),
            dist_theta(gen),
            1.0 / num_particles // inital weight
        };
        p.theta = normalize_angle(p.theta);
        particles.push_back(p);
    }
    is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
    
    default_random_engine gen;
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_theta(0, std_pos[2]);
    
    for (int i = 0; i < num_particles; i ++){
        
        // reference to the particle, modify inplace
        Particle& p = particles[i];
        
        double cos_theta = cos(p.theta);
        double sin_theta = sin(p.theta);
        
        if(fabs(yaw_rate) < 0.001){
            p.x += velocity * delta_t * cos_theta;
            p.y += velocity * delta_t * sin_theta;
        } else {
            double v_rawd = velocity / yaw_rate;
            double max_yaw = p.theta + yaw_rate * delta_t;
            p.x += v_rawd * (sin(max_yaw) - sin(p.theta));
            p.y += v_rawd * (cos(p.theta) - cos(max_yaw));
            p.theta += yaw_rate * delta_t;
        }
        
        // add noise
        p.x += dist_x(gen);
        p.y += dist_y(gen);
        p.theta += dist_theta(gen);
        p.theta = normalize_angle(p.theta);
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
    
    std::vector<Map::single_landmark_s> landmark_list = map_landmarks.landmark_list;
    
    for (int ii = 0; ii < num_particles; ii++){
        
        Particle& cur_p = particles[ii];
        double p_x = cur_p.x;
        double p_y = cur_p.y;
        double p_theta = cur_p.theta;
        
        std::vector<LandmarkObs> map_obs;
        
        for (int jj = 0; jj < observations.size(); jj ++){

            double ob_xc = observations[jj].x;
            double ob_yc = observations[jj].y;
            
            // transform observations to global map coordinate
            LandmarkObs ob_m;
            ob_m.x = p_x + cos(p_theta) * ob_xc - sin(p_theta) * ob_yc;
            ob_m.y = p_y + sin(p_theta) * ob_xc + cos(p_theta) * ob_yc;
            
            map_obs.push_back(ob_m);
        }//END_FOR JJ
        
        std::vector<Map::single_landmark_s> landmarks;
        // filter out those far away landmarks to improve data association speed
        for(int jj=0; jj < landmark_list.size(); jj++){
            double p_to_mark = dist(p_x, p_y, landmark_list[jj].x_f, landmark_list[jj].y_f);
            if( p_to_mark < sensor_range){
                landmarks.push_back(landmark_list[jj]);
            }
        }
        // What if landmarks.size() == 0 ?
        if (landmarks.size() == 0){
            cur_p.weight = 0.0;
            continue;
        }
        
        std::vector<int> associations;
        std::vector<double> sense_x;
        std::vector<double> sense_y;
        double weight = 1.0;
        
        // data association and meanwhile calculate the weight
        // allow one landmark been associate multiple times first
        for (int jj = 0; jj < map_obs.size(); jj++){
            
            double obs_x = map_obs[jj].x;
            double obs_y = map_obs[jj].y;
            
            // nearest neighbor to find the association
            int best_index = -1;
            double best_score = std::numeric_limits<double>::infinity();
            for(int kk = 0; kk < landmarks.size(); kk++){
                double score = dist(obs_x, obs_y, landmarks[kk].x_f, landmarks[kk].y_f);
                if( score < best_score){
                    best_index = kk;
                    best_score = score;
                }
            }
            
            int id_i = landmarks[best_index].id_i;
            double x_f = landmarks[best_index].x_f;
            double y_f = landmarks[best_index].y_f;
            
            weight *= prob(x_f, y_f, std_landmark[0], std_landmark[1], obs_x, obs_y);
            
            associations.push_back(id_i);
            sense_x.push_back(obs_x);
            sense_y.push_back(obs_y);
            
        }
        
        // update the weight and associations
        cur_p.weight = weight;
        SetAssociations(cur_p, associations, sense_x, sense_y);
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
    std::vector<double> weights;
    for(int i = 0; i < particles.size(); i++){
        weights.push_back(particles[i].weight);
    }
    
    default_random_engine gen;
    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> new_particles;
    for(int i = 0; i < particles.size(); i++){
        int index = d(gen);
        new_particles.push_back(particles[index]);
    }
    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
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
