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



#include <stdlib.h>
#include "gmcl/map/map.h"
#include "gmcl/pf/pf.h"

void map_update_espace(energy_map_t *energy_map , map_t *map ,lsensor_scan_t *laser_scan)     
{ 
  int range_num ;
  double map_range ;
  pf_vector_t pose;
      
  int step = (laser_scan->range_count - 1) / (energy_map->max_beams - 1);
   
   for (int j=0 ; j<energy_map->size_y ; j++ )
   {	
   	 for (int i=0 ; i<energy_map->size_x ; i++ )
       { 
          pose.v[0] = energy_map->poses[i + j *energy_map->size_x].pose_x ;
          pose.v[1] = energy_map->poses[i + j *energy_map->size_x].pose_y ;
          pose.v[2] = energy_map->poses[i + j *energy_map->size_x].orientation_yaw;
           // Take account of the laser pose relative to the robot
  	  pose = pf_vector_coord_add(laser_scan->laser_pose, pose);
    
  	  double p = 0 ;
  	  
  	  for (range_num = 0; range_num < laser_scan->range_count; range_num += step)
  	//for (range_num = 0; range_num < laser_scan->range_count; range_num ++)
         {    
              // Compute the range according to the map
             map_range = map_calc_range(map, pose.v[0], pose.v[1],
                                      pose.v[2] + laser_scan->ranges[range_num][1], laser_scan->range_max);
                                                
              p += 1 - map_range/laser_scan->range_max;
            }
            

          p /= laser_scan->range_count/step ;  
       //    p /= laser_scan->range_count ;  
        energy_map->poses[i + j *energy_map->size_x].energy_val  += p ; //energy in grid cell
     //   printf("energy_val: %9.6f\n", p); 
     }

    
  }

}  


void map_calc_SER(energy_map_t *energy_map , lsensor_scan_t *laser_scan) 
{ 
   int range_num , step ;
   double p = 0 ;
   double range ;
   step = (laser_scan->range_count - 1) / (energy_map->max_beams - 1);
            
  for (range_num = 0; range_num < laser_scan->range_count; range_num += step)
  // for (range_num = 0; range_num < laser_scan->range_count; range_num ++)
    {   
      range = laser_scan->ranges[range_num][0];
      if( range < laser_scan->range_max && range == range)
        p += 1 - range/laser_scan->range_max;
     }

     

   p /= laser_scan->range_count/step ;   
   //  p /= laser_scan->range_count ;   
   energy_map->lasers_energy_val +=p ;
  // printf("lasers_energy_val: %9.6f\n", p); 
}                                  


void map_clear_SER(energy_map_t *energy_map)
{ 
   
     energy_map->lasers_energy_val = 0.0 ;
}    


void map_diff_SER(energy_map_t *energy_map)
{ int accepted_count = 0 ;
  energy_pose_t* poses = energy_map->poses;
  int size = energy_map->size_x*energy_map->size_y ;
  int* accepted_index =  energy_map->accepted_index;
  
  for (int i=0 ; i<size ; i++ )  
      if(abs(energy_map->lasers_energy_val - poses[i].energy_val) < energy_map->threshold_val)                      
           accepted_index[accepted_count++] = i ;    
                               
  energy_map->accepted_count =  accepted_count ;  
             
  map_clear_SER(energy_map);                 
  // printf("accepted energy samples: %9.6d\n", accepted_count );    
}  
  

