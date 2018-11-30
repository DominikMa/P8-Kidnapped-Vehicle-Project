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

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  num_particles = 200;

  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  for (int i = 0; i < num_particles; ++i) {
    Particle p = Particle();
    p.id = i;
    p.x = dist_x(gen);
    p.y = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;
    particles.push_back(p);
  }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  normal_distribution<double> dist_x(0, std_pos[0]);
  normal_distribution<double> dist_y(0, std_pos[1]);
  normal_distribution<double> dist_theta(0, std_pos[2]);

  for (auto &p : particles) {
    if (yaw_rate != 0) {
      p.x += velocity / yaw_rate * (sin(p.theta + yaw_rate * delta_t) - sin(p.theta));
      p.y += velocity / yaw_rate * (cos(p.theta) - cos(p.theta + yaw_rate * delta_t));
      p.theta += yaw_rate * delta_t;
    } else {
      p.x += velocity * delta_t * cos(p.theta);
      p.y += velocity * delta_t * sin(p.theta);
    }
    p.x += dist_x(gen);
    p.y += dist_y(gen);
    p.theta += dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs> &observations) {
  for (auto &observation : observations) {
    double dist_obs = std::numeric_limits<double>::max();

    for (auto &prediction : predicted) {
      double d = dist(observation.x, observation.y, prediction.x, prediction.y);
      if (d < dist_obs) {
        observation.id = prediction.id;
        dist_obs = d;
      }
    }
  }

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
  double gauss_norm = 2 * M_PI * std_landmark[0] * std_landmark[1];

  for (auto &particle : particles) {

    //calculate predictions
    std::vector<LandmarkObs> predicted;
    for (auto &landmark : map_landmarks.landmark_list) {
      //double dist = sqrt(pow(particle.x-landmark.x_f,2)+pow(particle.y-landmark.y_f,2));
      double d = dist(particle.x, particle.y, landmark.x_f, landmark.y_f);
      if (d <= sensor_range) {
        LandmarkObs l;
        l.id = landmark.id_i;
        l.x = landmark.x_f;
        l.y = landmark.y_f;
        predicted.push_back(l);
      }
    }

    //map observations
    std::vector<LandmarkObs> observations_trans;
    for (auto &observation : observations) {
      LandmarkObs l;
      l.x = particle.x + cos(particle.theta) * observation.x - sin(particle.theta) * observation.y;
      l.y = particle.y + sin(particle.theta) * observation.x + cos(particle.theta) * observation.y;
      l.id = -1;
      observations_trans.push_back(l);
    }

    //calculate associations
    dataAssociation(predicted, observations_trans);

    //calculate probabilities
    for (auto &observation : observations_trans) {
      for (auto &landmark : predicted) {
        if (observation.id == landmark.id) {
          double exponent = exp(-(pow(observation.x - landmark.x, 2) / (2 * pow(std_landmark[0], 2))
              + pow(observation.y - landmark.y, 2) / (2 * pow(std_landmark[1], 2))));
          if (exponent != 0){
            particle.weight *= exponent / gauss_norm;
          }
          break;
        }
      }
    }
  }
}

void ParticleFilter::resample() {
  vector<double> old_weights;
  for (auto &particle : particles) {
    old_weights.push_back(particle.weight);
  }
  std::discrete_distribution<int> d(old_weights.begin(), old_weights.end());

  vector<Particle> new_particles;
  for (int i = 0; i < num_particles; ++i) {
    Particle p = particles[d(gen)];
    p.weight = 1.0;
    new_particles.push_back(p);
  }
  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle &particle, const std::vector<int> &associations,
                                         const std::vector<double> &sense_x, const std::vector<double> &sense_y) {
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
