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
 * CVS: $Id: map.h 1713 2003-08-23 04:03:43Z inspectorg $
 **************************************************************************/

#ifndef MAP_H
#define MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct _rtk_fig_t;
 
// Limits
#define MAP_WIFI_MAX_LEVELS 8
  
// Description for a single map cell.
typedef struct
{
  // Occupancy state (-1 = free, 0 = unknown, +1 = occ)
  int occ_state;

  // Distance to the nearest occupied cell
  double occ_dist;
   
  // Wifi levels
  //int wifi_levels[MAP_WIFI_MAX_LEVELS];
  
} map_cell_t;


// Description for a map
typedef struct
{
  // Map origin; the map is a viewport onto a conceptual larger map.
  double origin_x, origin_y;
  
  // Map scale (m/cell)
  double scale;

  // Map dimensions (number of cells)
  int size_x, size_y;
  
  // The map data, stored as a grid
  map_cell_t *cells;

  // Max distance at which we care about obstacles, for constructing
  // likelihood field
  double max_occ_dist;
  
} map_t;

#include "gmcl/pf/pf.h"
typedef struct
{
   pf_vector_t laser_pose ;
  
   int range_count ;
   
   float range_max ;
   
   float range_min ;
        
   double (*ranges)[2] ; 
       
} lsensor_scan_t;


typedef struct
{
  // pose of robot in the map.
  double pose_x, pose_y , orientation_yaw;
    
  // energy stored in pose 
  double energy_val ;
  
  
} energy_pose_t;

typedef struct
{
  double min_x , max_x , min_y , max_y , laser_min_range ,laser_max_range;
  
  energy_pose_t *poses ;
  
  double resoultion_x , resoultion_y  ;
    
  int size_x , size_y , max_beams , accepted_count ;   
  
  int* accepted_index ;
  
  double lasers_energy_val ,threshold_val ;
     
 
} energy_map_t;

/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map
map_t *map_alloc(void);

// Create a new (empty) map
energy_map_t *energy_map_alloc(map_t *map, double resoultion_x , double resoultion_y, 
                                               size_t max_beams , double laser_min_range ,double laser_max_range);
// Destroy a map
void map_free(map_t *map);

// Destroy a map
void energy_map_free(energy_map_t *energy_map);

// Get the cell at the given point
map_cell_t *map_get_cell(map_t *map, double ox, double oy, double oa);

// Load an occupancy map
int map_load_occ(map_t *map, const char *filename, double scale, int negate);

// Load a wifi signal strength map
//int map_load_wifi(map_t *map, const char *filename, int index);

// Update the cspace distances
void map_update_cspace(map_t *map, double max_occ_dist);

// Update the espace distances

void map_update_espace(energy_map_t *energy_map , map_t *map , lsensor_scan_t *laser_scan);         
                                  
void map_calc_SER(energy_map_t *energy_map , lsensor_scan_t *laser_scan);     

void map_diff_SER(energy_map_t *energy_map);

void map_clear_SER(energy_map_t *energy_map);   
   
/**************************************************************************
 * Range functions
 **************************************************************************/

// Extract a single range reading from the map
double map_calc_range(map_t *map, double ox, double oy, double oa, double max_range);


/**************************************************************************
 * GUI/diagnostic functions
 **************************************************************************/

// Draw the occupancy grid
void map_draw_occ(map_t *map, struct _rtk_fig_t *fig);

// Draw the cspace map
void map_draw_cspace(map_t *map, struct _rtk_fig_t *fig);

// Draw a wifi map
void map_draw_wifi(map_t *map, struct _rtk_fig_t *fig, int index);


/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords
#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

// Convert from world coords to map coords
#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

// Test to see if the given map coords lie within the absolute map bounds.
#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

// Compute the cell index for the given map coords.
#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif
