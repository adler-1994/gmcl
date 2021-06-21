//this package is based on amcl and has been modified to fit gmcl 
/* 
 * Author: Mhd Ali Alshikh Khalil
 * Date: 20 June 2021
 * 
*/

//amcl author clarification
/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL laser routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_laser.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////
#include "ros/ros.h"
#include <sys/types.h> // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#include "gmcl/sensors/gmcl_laser.h"

using namespace gmcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
GMCLLaser::GMCLLaser(size_t max_beams, map_t* map) : GMCLSensor(), 
						     max_samples(0), max_obs(0), 
						     temp_obs(NULL)
{
  this->time = 0.0;

  this->max_beams = max_beams;
  this->map = map;

  return;
}

GMCLLaser::~GMCLLaser()
{
  if(temp_obs){
	for(int k=0; k < max_samples; k++){
	  delete [] temp_obs[k];
	}
	delete []temp_obs; 
  }
}

void 
GMCLLaser::SetModelBeam(double z_hit,
                        double z_short,
                        double z_max,
                        double z_rand,
                        double sigma_hit,
                        double lambda_short,
                        double chi_outlier)
{
  this->model_type = LASER_MODEL_BEAM;
  this->z_hit = z_hit;
  this->z_short = z_short;
  this->z_max = z_max;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->lambda_short = lambda_short;
  this->chi_outlier = chi_outlier;
}

void 
GMCLLaser::SetModelLikelihoodField(double z_hit,
                                   double z_rand,
                                   double sigma_hit,
                                   double max_occ_dist)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;

  map_update_cspace(this->map, max_occ_dist);
}

void 
GMCLLaser::SetModelLikelihoodFieldProb(double z_hit,
				       double z_rand,
				       double sigma_hit,
				       double max_occ_dist,
				       bool do_beamskip,
				       double beam_skip_distance,
				       double beam_skip_threshold, 
				       double beam_skip_error_threshold)
{
  this->model_type = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->do_beamskip = do_beamskip;
  this->beam_skip_distance = beam_skip_distance;
  this->beam_skip_threshold = beam_skip_threshold;
  this->beam_skip_error_threshold = beam_skip_error_threshold;
  map_update_cspace(this->map, max_occ_dist);
}


////////////////////////////////////////////////////////////////////////////////
// Apply the laser sensor model
bool GMCLLaser::UpdateSensor(pf_t *pf, GMCLSensorData *data) 
{
  if (this->max_beams < 2)
    return false;

  // Apply the laser sensor model
  if(this->model_type == LASER_MODEL_BEAM)
    pf_update_sensor(pf, (pf_laser_model_fn_t) BeamModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD)
    pf_update_sensor(pf, (pf_laser_model_fn_t) LikelihoodFieldModel, data);
  else if(this->model_type == LASER_MODEL_LIKELIHOOD_FIELD_PROB)
    pf_update_sensor(pf, (pf_laser_model_fn_t) LikelihoodFieldModelProb, data);  
  else
    pf_update_sensor(pf, (pf_laser_model_fn_t) BeamModel, data);

  return true;
}





double GMCLLaser::PoseWeight( pf_vector_t pose, GMCLLaserData *data) 
{
    GMCLLaser *self;
    int i, j, step;
    double z, pz;
    double p;
    double map_range;
    double obs_range, obs_bearing;
    double total_weight;

    self = (GMCLLaser*) data->sensor;
    
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, self->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == self->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < self->range_max)
        pz += self->z_rand * 1.0/self->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    
   return p;

}
////////////////////////////////////////////////////////////////////////////////
// Determine the probability for the given pose
void GMCLLaser::BeamModel(GMCLLaserData *data, pf_sample_set_t* set) 
{
  GMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double map_range;
  double obs_range, obs_bearing;
  double total_weight;
  pf_t *pf ;
  pf_sample_t *sample, *aux_sample;
  pf_vector_t pose;
  double group_weight;
  self = (GMCLLaser*) data->sensor;

  pf = set->pf ;
  

  int updated_count = set->updated_count;
  int* updated_index = set->updated_index;

  double range_max = self->range_max;
  int max_beams = self->max_beams;
  int range_count = data->range_count;
  if(updated_count > 0)  
  {// Compute the sample weights
    for (j = 0; j < updated_count; j++)
    {
      sample = set->samples + updated_index[j];
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, self->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == self->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < self->range_max)
        pz += self->z_rand * 1.0/self->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    sample->weight *= p;

    } 
   printf("Num of crossovered samples: %9.6d\n", updated_count); 
    }
 else  
   if(pf->model.use_optimal_filter)
  // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    { group_weight = 0.0 ;
      sample = set->samples + j;
        for ( int loop=0 ; loop < pf->N_aux_particles ; loop++)
      {
        aux_sample = set->aux_samples + (j*pf->N_aux_particles+loop) ;
        pose = aux_sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, self->range_max);
      pz = 0.0;

      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == self->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < self->range_max)
        pz += self->z_rand * 1.0/self->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    aux_sample->weight = p ;
    group_weight += aux_sample->weight ;
    }
    sample->weight *= group_weight/pf->N_aux_particles;
    sample->last_obs = group_weight/pf->N_aux_particles;
    //printf(" group weight %f ", group_weight );
    }       
  else if(pf->model.use_crossover_mutation)
  // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, self->range_max);
      pz = 0.0;
 
      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == self->range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < self->range_max)
        pz += self->z_rand * 1.0/self->range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;

    sample->last_obs = p; 
    } 
  else
    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (range_count - 1) / (max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // Compute the range according to the map
      map_range = map_calc_range(self->map, pose.v[0], pose.v[1],
                                 pose.v[2] + obs_bearing, range_max);
      pz = 0.0;
//     printf("obs_range: %9.6f\n", obs_range); 
       //         printf("range_max: %9.6f\n", range_max);      
            //       printf("max_beams: %9.6d\n", max_beams );
          //                             printf("range_count: %9.6d\n", range_count  );
      // Part 1: good, but noisy, hit
      z = obs_range - map_range;
      pz += self->z_hit * exp(-(z * z) / (2 * self->sigma_hit * self->sigma_hit));

      // Part 2: short reading from unexpected obstacle (e.g., a person)
      if(z < 0)
        pz += self->z_short * self->lambda_short * exp(-self->lambda_short*obs_range);

      // Part 3: Failure to detect obstacle, reported as max-range
      if(obs_range == range_max)
        pz += self->z_max * 1.0;

      // Part 4: Random measurements
      if(obs_range < range_max)
        pz += self->z_rand * 1.0/range_max;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    
    }

    sample->weight *= p;
    }   
  

}

void GMCLLaser::LikelihoodFieldModel(GMCLLaserData *data, pf_sample_set_t* set) 
{
 
  GMCLLaser *self;
  int i, j, step;
  double z, pz;
  double p;
  double obs_range, obs_bearing;
  double total_weight;
  pf_t *pf;
  pf_sample_t *sample, *aux_sample;
  pf_vector_t pose;
  pf_vector_t hit;
  double group_weight;
  
  pf = set->pf ;

  self = (GMCLLaser*) data->sensor;
  
 int updated_count = set->updated_count;
 int* updated_index = set->updated_index;
 
 double range_max = self->range_max;
 int max_beams = self->max_beams;
 int range_count = data->range_count; 
  // Pre-compute a couple of things
 double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
 double z_rand_mult = 1.0/range_max;
 
 
if(updated_count > 0)  
  {// Compute the sample weights
    for (j = 0; j < updated_count; j++)
    {
      sample = set->samples + updated_index[j];
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];
      
      if(obs_range >= range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;
      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    sample->weight *= p;

    } 
   printf("Num of crossovered samples: %9.6d\n", updated_count); 
    }
 else  
   if(pf->model.use_optimal_filter)
  // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    { group_weight = 0.0 ;
      sample = set->samples + j;
        for ( int loop=0 ; loop < pf->N_aux_particles ; loop++)
      {
        aux_sample = set->aux_samples + (j*pf->N_aux_particles+loop) ;
        pose = aux_sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];
      
      if(obs_range >= range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;


      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }
    aux_sample->weight = p ;
    group_weight += aux_sample->weight ;
    }
    sample->weight *= group_weight/pf->N_aux_particles;
    sample->last_obs = group_weight/pf->N_aux_particles;
    //printf(" group weight %f ", group_weight );
    }       
  else if(pf->model.use_crossover_mutation)
  // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (data->range_count - 1) / (self->max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;
 
      // Part 1: good, but noisy, hit
     // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    }

    sample->weight *= p;

    sample->last_obs = p; 
    } 
  else
    for (j = 0; j < set->sample_count; j++)
    {
      sample = set->samples + j;
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    p = 1.0;

    step = (range_count - 1) / (max_beams - 1);
    for (i = 0; i < data->range_count; i += step)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      if(obs_range >= range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

     // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj))
        z = self->map->max_occ_dist;
      else
        z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz*pz*pz;
    
    }

    sample->weight *= p;
    }   
    

}
void GMCLLaser::LikelihoodFieldModelProb(GMCLLaserData *data, pf_sample_set_t* set) 
{
  GMCLLaser *self;
  int i, j, step;
  double z, pz;
  double log_p;
  double obs_range, obs_bearing;
  double group_weight;

  pf_sample_t *sample, *aux_sample;;
  pf_vector_t pose;
  pf_vector_t hit;

  self = (GMCLLaser*) data->sensor;

  pf_t *pf;
  
  pf = set->pf ;
  
 int  updated_count = set->updated_count;
 int* updated_index = set->updated_index;
 
 double range_max = self->range_max;
 int max_beams = self->max_beams;
 int range_count = data->range_count; 

  step = ceil((data->range_count) / static_cast<double>(self->max_beams)); 
  
  // Step size must be at least 1
  if(step < 1)
    step = 1;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;
  double z_rand_mult = 1.0/self->range_max;

  double max_dist_prob = exp(-(self->map->max_occ_dist * self->map->max_occ_dist) / z_hit_denom);

  //Beam skipping - ignores beams for which a majoirty of particles do not agree with the map
  //prevents correct particles from getting down weighted because of unexpected obstacles 
  //such as humans 

  bool do_beamskip = self->do_beamskip;
  double beam_skip_distance = self->beam_skip_distance;
  double beam_skip_threshold = self->beam_skip_threshold;
  
  //we only do beam skipping if the filter has converged 
  if(do_beamskip && !set->converged){
    do_beamskip = false;
  }

  //we need a count the no of particles for which the beam agreed with the map 
  int *obs_count = new int[self->max_beams]();

  //we also need a mask of which observations to integrate (to decide which beams to integrate to all particles) 
  bool *obs_mask = new bool[self->max_beams]();
  
  int beam_ind = 0;
  
  //realloc indicates if we need to reallocate the temp data structure needed to do beamskipping 
  bool realloc = false; 

  if(do_beamskip && updated_count == 0){
    if(self->max_obs < self->max_beams){
      realloc = true;
    }

    if(self->max_samples < set->sample_count){
      realloc = true;
    }

    if(realloc){
      self->reallocTempData(set->sample_count, self->max_beams);     
      fprintf(stderr, "Reallocing temp weights %d - %d\n", self->max_samples, self->max_obs);
    }
  }
 
if(updated_count > 0)  
  {// Compute the sample weights
    for (j = 0; j < updated_count; j++)
    {
      sample = set->samples + updated_index[j];
      pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;    
    beam_ind = 0;

    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];
      
      if(obs_range >= range_max)
        continue;

      // Check for NaN
      if(obs_range != obs_range)
        continue;

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
     
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;
      // TODO: outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(updated_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }
   
      for (j = 0; j < updated_count; j++)
      {
       sample = set->samples + updated_index[j];
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);
	
      }      
  }

   printf("Num of crossovered samples: %9.6d\n", updated_count); 
    }  
  
 else  
   if(pf->model.use_optimal_filter)
 { // Compute the sample weights
    for (j = 0; j < set->sample_count; j++)
    { group_weight = 0.0 ;
      sample = set->samples + j;
        for ( int loop=0 ; loop < pf->N_aux_particles ; loop++)
      {
        aux_sample = set->aux_samples + (j*pf->N_aux_particles+loop) ;
        pose = aux_sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
       obs_range = data->ranges[i][0];
       obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      aux_sample->weight = exp(log_p);
      group_weight += aux_sample->weight ;
      } 
    }
    if(!do_beamskip){
      sample->weight *= group_weight/pf->N_aux_particles;
      sample->last_obs = group_weight/pf->N_aux_particles;  
      }
  }
  
   if(do_beamskip){
     int skipped_beam_count = 0; 
     for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
     { group_weight = 0.0 ;
      sample = set->samples + j;
        for ( int loop=0 ; loop < pf->N_aux_particles ; loop++)
      {
        aux_sample = set->aux_samples + (j*pf->N_aux_particles+loop) ;
        pose = aux_sample->pose;
  
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	aux_sample->weight = exp(log_p);
       group_weight += aux_sample->weight ;

      } 
      sample->weight *= group_weight/pf->N_aux_particles;
      sample->last_obs = group_weight/pf->N_aux_particles;       
    } 
  
  }
  }
    
  else if(pf->model.use_crossover_mutation)
 { // Compute the sample weights
   for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
      sample->last_obs = exp(log_p); 
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
      {
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);
        sample->last_obs = exp(log_p); 

      }      
  }

} 
  else
  {   for (j = 0; j < set->sample_count; j++)
  {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose, pose);

    log_p = 0;
    
    beam_ind = 0;
    
    for (i = 0; i < data->range_count; i += step, beam_ind++)
    {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if(obs_range >= range_max){
        continue;
      }

      // Check for NaN
      if(obs_range != obs_range){
        continue;
      }

      pz = 0.0;

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map, hit.v[0]);
      mj = MAP_GYWY(self->map, hit.v[1]);
      
      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      
      if(!MAP_VALID(self->map, mi, mj)){
	pz += self->z_hit * max_dist_prob;
      }
      else{
	z = self->map->cells[MAP_INDEX(self->map,mi,mj)].occ_dist;
	if(z < beam_skip_distance){
	  obs_count[beam_ind] += 1;
	}
	pz += self->z_hit * exp(-(z * z) / z_hit_denom);
      }
       
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      
      // Part 2: random measurements
      pz += self->z_rand * z_rand_mult;

      assert(pz <= 1.0); 
      assert(pz >= 0.0);

      // TODO: outlier rejection for short readings
            
      if(!do_beamskip){
	log_p += log(pz);
      }
      else{
	self->temp_obs[j][beam_ind] = pz; 
      }
    }
    if(!do_beamskip){
      sample->weight *= exp(log_p);
    }
  }
  
  if(do_beamskip){
    int skipped_beam_count = 0; 
    for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
      if((obs_count[beam_ind] / static_cast<double>(set->sample_count)) > beam_skip_threshold){
	obs_mask[beam_ind] = true;
      }
      else{
	obs_mask[beam_ind] = false;
	skipped_beam_count++; 
      }
    }

    //we check if there is at least a critical number of beams that agreed with the map 
    //otherwise it probably indicates that the filter converged to a wrong solution
    //if that's the case we integrate all the beams and hope the filter might converge to 
    //the right solution
    bool error = false; 

    if(skipped_beam_count >= (beam_ind * self->beam_skip_error_threshold)){
      fprintf(stderr, "Over %f%% of the observations were not in the map - pf may have converged to wrong pose - integrating all observations\n", (100 * self->beam_skip_error_threshold));
      error = true; 
    }

    for (j = 0; j < set->sample_count; j++)
      {
	sample = set->samples + j;
	pose = sample->pose;

	log_p = 0;

	for (beam_ind = 0; beam_ind < self->max_beams; beam_ind++){
	  if(error || obs_mask[beam_ind]){
	    log_p += log(self->temp_obs[j][beam_ind]);
	  }
	}
	
	sample->weight *= exp(log_p);

      }      
  }

}

  delete [] obs_count; 
  delete [] obs_mask;
}  


void GMCLLaser::reallocTempData(int new_max_samples, int new_max_obs){
  if(temp_obs){
    for(int k=0; k < max_samples; k++){
      delete [] temp_obs[k];
    }
    delete []temp_obs; 
  }
  max_obs = new_max_obs; 
  max_samples = fmax(max_samples, new_max_samples); 

  temp_obs = new double*[max_samples]();
  for(int k=0; k < max_samples; k++){
    temp_obs[k] = new double[max_obs]();
  }
}
