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
// Desc: AMCL odometry routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_odom.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <algorithm>
#include "ros/ros.h"
#include <sys/types.h> // required by Darwin
#include <math.h>

#include "gmcl/sensors/gmcl_odom.h"

using namespace gmcl;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

////////////////////////////////////////////////////////////////////////////////
// Default constructor
GMCLOdom::GMCLOdom() : GMCLSensor()
{
  this->time = 0.0;
}

void
GMCLOdom::SetModelDiff(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4)
{
  this->model_type = ODOM_MODEL_DIFF;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
}

void
GMCLOdom::SetModelOmni(double alpha1, 
                       double alpha2, 
                       double alpha3, 
                       double alpha4,
                       double alpha5)
{
  this->model_type = ODOM_MODEL_OMNI;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

void
GMCLOdom::SetModel( odom_model_t type,
                    double alpha1,
                    double alpha2,
                    double alpha3,
                    double alpha4,
                    double alpha5 )
{
  this->model_type = type;
  this->alpha1 = alpha1;
  this->alpha2 = alpha2;
  this->alpha3 = alpha3;
  this->alpha4 = alpha4;
  this->alpha5 = alpha5;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool GMCLOdom::UpdateAction(pf_t *pf, GMCLSensorData *data)
{   
  // Apply the laser sensor model
  if(this->model_type ==  ODOM_MODEL_OMNI)
    pf_update_action(pf, (pf_action_model_fn_t) OmniModel, data);
  else if(this->model_type == ODOM_MODEL_OMNI_CORRECTED)
    pf_update_action(pf, (pf_action_model_fn_t) OmniCorrectedModel, data);  
  else if(this->model_type == ODOM_MODEL_DIFF)
    pf_update_action(pf, (pf_action_model_fn_t) DiffModel, data);  
  else if(this->model_type == ODOM_MODEL_DIFF_CORRECTED)
    pf_update_action(pf, (pf_action_model_fn_t) DiffCorrectedModel, data);  
  else
    pf_update_action(pf, (pf_action_model_fn_t) DiffModel, data);

  return true;
}

 
void GMCLOdom::OmniModel(GMCLOdomData *data , pf_sample_set_t* set)
 {  GMCLOdom *self;
    self = (GMCLOdom*) data->sensor;
    pf_t *pf;
    pf_sample_t* aux_sample;
    pf = set->pf;
    
    // Compute the new sample poses
    pf_vector_t old_pose = pf_vector_sub(data->pose, data->delta);

    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(data->delta.v[0]*data->delta.v[0] +
                       data->delta.v[1]*data->delta.v[1]);
    delta_rot = data->delta.v[2];

    // Precompute a couple of things
    double trans_hat_stddev = (self->alpha3 * (delta_trans*delta_trans) +
                               self->alpha1 * (delta_rot*delta_rot));
    double rot_hat_stddev = (self->alpha4 * (delta_rot*delta_rot) +
                             self->alpha2 * (delta_trans*delta_trans));
    double strafe_hat_stddev = (self->alpha1 * (delta_rot*delta_rot) +
                                self->alpha5 * (delta_trans*delta_trans));

 if(pf->model.use_optimal_filter)
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;
      delta_bearing = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);
      for (int j = 0; j < pf->N_aux_particles; j++)
       {
         aux_sample = set->aux_samples + (i*pf->N_aux_particles+j) ;
       
         // Sample pose differences
         delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
         delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
         delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);

        // Apply sampled update to particle pose
        aux_sample->pose.v[0] = (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing) + sample->pose.v[0];
        aux_sample->pose.v[1] = (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing) + sample->pose.v[1];
        aux_sample->pose.v[2] = delta_rot_hat + sample->pose.v[2];
    }
    
   }
else
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
  
  
  
  
void GMCLOdom::DiffModel(GMCLOdomData *data , pf_sample_set_t* set)
  { GMCLOdom *self;
    self = (GMCLOdom*) data->sensor ;
    pf_t *pf;
    pf_sample_t* aux_sample;
    pf = set->pf;
    // Compute the new sample poses

    pf_vector_t old_pose = pf_vector_sub(data->pose, data->delta);

    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(data->delta.v[1]*data->delta.v[1] + 
            data->delta.v[0]*data->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
    delta_rot1 = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(data->delta.v[0]*data->delta.v[0] +
                       data->delta.v[1]*data->delta.v[1]);
    delta_rot2 = angle_diff(data->delta.v[2], delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));

 if(pf->model.use_optimal_filter)
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;
      for (int j = 0; j < pf->N_aux_particles; j++)
       {
         aux_sample = set->aux_samples + (i*pf->N_aux_particles+j) ;

         // Sample pose differences
         delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(self->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                  self->alpha2*delta_trans*delta_trans));
         delta_trans_hat = delta_trans - 
              pf_ran_gaussian(self->alpha3*delta_trans*delta_trans +
                              self->alpha4*delta_rot1_noise*delta_rot1_noise +
                              self->alpha4*delta_rot2_noise*delta_rot2_noise);
         delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(self->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                  self->alpha2*delta_trans*delta_trans));

        // Apply sampled update to particle pose
        aux_sample->pose.v[0] = delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat) + sample->pose.v[0];
        aux_sample->pose.v[1] = delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat) + sample->pose.v[1];
        aux_sample->pose.v[2] = delta_rot1_hat + delta_rot2_hat + sample->pose.v[2];
    }

   }
 else
   for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(self->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                  self->alpha2*delta_trans*delta_trans));
      delta_trans_hat = delta_trans - 
              pf_ran_gaussian(self->alpha3*delta_trans*delta_trans +
                              self->alpha4*delta_rot1_noise*delta_rot1_noise +
                              self->alpha4*delta_rot2_noise*delta_rot2_noise);
      delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(self->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                  self->alpha2*delta_trans*delta_trans));

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
    }

  }

  void GMCLOdom::OmniCorrectedModel(GMCLOdomData *data , pf_sample_set_t* set)
  { GMCLOdom *self;
    self = (GMCLOdom*) data->sensor;
    pf_t *pf;
    pf_sample_t* aux_sample;
    pf = set->pf; 
    // Compute the new sample poses
    pf_vector_t old_pose = pf_vector_sub(data->pose, data->delta);
    double delta_trans, delta_rot, delta_bearing;
    double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

    delta_trans = sqrt(data->delta.v[0]*data->delta.v[0] +
                       data->delta.v[1]*data->delta.v[1]);
    delta_rot = data->delta.v[2];
    // Precompute a couple of things
    double trans_hat_stddev = sqrt( self->alpha3 * (delta_trans*delta_trans) +
                                    self->alpha4 * (delta_rot*delta_rot) );
    double rot_hat_stddev = sqrt( self->alpha1 * (delta_rot*delta_rot) +
                                  self->alpha2 * (delta_trans*delta_trans) );
    double strafe_hat_stddev = sqrt( self->alpha4 * (delta_rot*delta_rot) +
                                     self->alpha5 * (delta_trans*delta_trans) );
  if(pf->model.use_optimal_filter)
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;
      delta_bearing = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);
      for (int j = 0; j < pf->N_aux_particles; j++)
       {
         aux_sample = set->aux_samples + (i*pf->N_aux_particles+j) ;
    
         // Sample pose differences
        delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
        delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
        delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);

        // Apply sampled update to particle pose
        aux_sample->pose.v[0] = (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing) + sample->pose.v[0];
        aux_sample->pose.v[1] = (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing) + sample->pose.v[1];
        aux_sample->pose.v[2] = delta_rot_hat + sample->pose.v[2];
    }
    
   }
else
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      delta_bearing = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                                 old_pose.v[2]) + sample->pose.v[2];
      double cs_bearing = cos(delta_bearing);
      double sn_bearing = sin(delta_bearing);

      // Sample pose differences
      delta_trans_hat = delta_trans + pf_ran_gaussian(trans_hat_stddev);
      delta_rot_hat = delta_rot + pf_ran_gaussian(rot_hat_stddev);
      delta_strafe_hat = 0 + pf_ran_gaussian(strafe_hat_stddev);
      // Apply sampled update to particle pose
      sample->pose.v[0] += (delta_trans_hat * cs_bearing + 
                            delta_strafe_hat * sn_bearing);
      sample->pose.v[1] += (delta_trans_hat * sn_bearing - 
                            delta_strafe_hat * cs_bearing);
      sample->pose.v[2] += delta_rot_hat ;
    }
  }
void GMCLOdom::DiffCorrectedModel(GMCLOdomData *data , pf_sample_set_t* set)
  { GMCLOdom *self;
    self = (GMCLOdom*) data->sensor;
    pf_t *pf;
    pf_sample_t* aux_sample;
    pf = set->pf;
    // Compute the new sample poses

    pf_vector_t old_pose = pf_vector_sub(data->pose, data->delta);
    // Implement sample_motion_odometry (Prob Rob p 136)
    double delta_rot1, delta_trans, delta_rot2;
    double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
    double delta_rot1_noise, delta_rot2_noise;

    // Avoid computing a bearing from two poses that are extremely near each
    // other (happens on in-place rotation).
    if(sqrt(data->delta.v[1]*data->delta.v[1] + 
            data->delta.v[0]*data->delta.v[0]) < 0.01)
      delta_rot1 = 0.0;
    else
      delta_rot1 = angle_diff(atan2(data->delta.v[1], data->delta.v[0]),
                              old_pose.v[2]);
    delta_trans = sqrt(data->delta.v[0]*data->delta.v[0] +
                       data->delta.v[1]*data->delta.v[1]);
    delta_rot2 = angle_diff(data->delta.v[2], delta_rot1);

    // We want to treat backward and forward motion symmetrically for the
    // noise model to be applied below.  The standard model seems to assume
    // forward motion.
    delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                                fabs(angle_diff(delta_rot1,M_PI)));
    delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                                fabs(angle_diff(delta_rot2,M_PI)));
if(pf->model.use_optimal_filter)
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;
      for (int j = 0; j < pf->N_aux_particles; j++)
       {
         aux_sample = set->aux_samples + (i*pf->N_aux_particles+j) ;

         // Sample pose differences
          delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(sqrt(self->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                       self->alpha2*delta_trans*delta_trans)));
          delta_trans_hat = delta_trans - 
                     pf_ran_gaussian(sqrt(self->alpha3*delta_trans*delta_trans +
                                   self->alpha4*delta_rot1_noise*delta_rot1_noise +
                                   self->alpha4*delta_rot2_noise*delta_rot2_noise));
          delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(sqrt(self->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                       self->alpha2*delta_trans*delta_trans)));

        // Apply sampled update to particle pose
        aux_sample->pose.v[0] = delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat) + sample->pose.v[0];
        aux_sample->pose.v[1] = delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat) + sample->pose.v[1];
        aux_sample->pose.v[2] = delta_rot1_hat + delta_rot2_hat + sample->pose.v[2];
    }

   }
 else
    for (int i = 0; i < set->sample_count; i++)
    {
      pf_sample_t* sample = set->samples + i;

      // Sample pose differences
      delta_rot1_hat = angle_diff(delta_rot1,
                                  pf_ran_gaussian(sqrt(self->alpha1*delta_rot1_noise*delta_rot1_noise +
                                                       self->alpha2*delta_trans*delta_trans)));
      delta_trans_hat = delta_trans - 
              pf_ran_gaussian(sqrt(self->alpha3*delta_trans*delta_trans +
                                   self->alpha4*delta_rot1_noise*delta_rot1_noise +
                                   self->alpha4*delta_rot2_noise*delta_rot2_noise));
      delta_rot2_hat = angle_diff(delta_rot2,
                                  pf_ran_gaussian(sqrt(self->alpha1*delta_rot2_noise*delta_rot2_noise +
                                                       self->alpha2*delta_trans*delta_trans)));

      // Apply sampled update to particle pose
      sample->pose.v[0] += delta_trans_hat * 
              cos(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[1] += delta_trans_hat * 
              sin(sample->pose.v[2] + delta_rot1_hat);
      sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
    }
  }
  
  


