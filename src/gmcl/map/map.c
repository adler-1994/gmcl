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
/**************************************************************************
 * Desc: Global map (grid-based)
 * Author: Andrew Howard
 * Date: 6 Feb 2003
 * CVS: $Id: map.c 1713 2003-08-23 04:03:43Z inspectorg $
**************************************************************************/

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "gmcl/map/map.h"


// Create a new map
map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  // Assume we start at (0, 0)
  map->origin_x = 0;
  map->origin_y = 0;
  
  // Make the size odd
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  // Allocate storage for main map
  map->cells = (map_cell_t*) NULL;
  
  return map;
}


energy_map_t *energy_map_alloc(map_t *map , double resoultion_x , double resoultion_y ,
                                      size_t max_beams, double laser_min_range ,double laser_max_range)
{
  energy_map_t *energy_map;

  energy_map = (energy_map_t*) malloc(sizeof(energy_map_t));

  // Assume we start at (0, 0)
  energy_map->min_x = map->origin_x - (map->size_x /2) * map->scale;
  energy_map->max_x = map->origin_x + (map->size_x /2) * map->scale;
  energy_map->min_y = map->origin_y - (map->size_y /2) * map->scale;
  energy_map->max_y = map->origin_y + (map->size_y /2) * map->scale;
  // Make the size odd
  energy_map->size_x =  abs(floor((energy_map->max_x - energy_map->min_x)/resoultion_x))+1 ;
  energy_map->size_y =  abs(floor((energy_map->max_y - energy_map->min_y)/resoultion_y))+1 ;     
                                    
  energy_map->resoultion_x = resoultion_x;
  energy_map->resoultion_y = resoultion_y;
  
  energy_map->laser_min_range = laser_min_range;
  energy_map->laser_max_range = laser_max_range;
  
  energy_map->max_beams = max_beams ;
  
  energy_map->lasers_energy_val = 0.0 ;

  // Allocate storage for main map
  
  energy_map->poses = (energy_pose_t*) NULL;
          
  energy_map->poses = calloc( energy_map->size_x *  energy_map->size_y, sizeof(energy_map->poses[0]));
  
  assert(energy_map->poses);
  
  energy_map->accepted_index = calloc (energy_map->size_x *  energy_map->size_y, sizeof(int));
  
  assert(energy_map->accepted_index);
  
  for (int j=0 ; j<energy_map->size_y ; j++ )
   {	
   	 for (int i=0 ; i<energy_map->size_x ; i++ )
       { 
           energy_map->poses[i + j *energy_map->size_x].pose_x = energy_map->min_x +(i *resoultion_x);
           energy_map->poses[i + j *energy_map->size_x].pose_y = energy_map->min_y +(j *resoultion_y);
           energy_map->poses[i + j *energy_map->size_x].orientation_yaw = drand48() * 2 * M_PI - M_PI ;
       }
  
   }
  
  return energy_map;
}


// Destroy a map
void map_free(map_t *map)
{
  free(map->cells);
  free(map);
  return;
}

void energy_map_free(energy_map_t *energy_map)
{
  free(energy_map->poses);
  free(energy_map->accepted_index);
  free(energy_map);
  return;
}
// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa)
{
  int i, j;
  map_cell_t *cell;

  i = MAP_GXWX(map, ox);
  j = MAP_GYWY(map, oy);
  
  if (!MAP_VALID(map, i, j))
    return NULL;

  cell = map->cells + MAP_INDEX(map, i, j);
  return cell;
}

