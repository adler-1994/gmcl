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
// Desc: AMCL sensor
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_sensor.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////


#include "gmcl/sensors/gmcl_sensor.h"

using namespace gmcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
GMCLSensor::GMCLSensor()
{
  return;
}

GMCLSensor::~GMCLSensor()
{
}

////////////////////////////////////////////////////////////////////////////////
// Apply the action model
bool GMCLSensor::UpdateAction(pf_t *pf, GMCLSensorData *data)
{
  return false;
}


////////////////////////////////////////////////////////////////////////////////
// Initialize the filter
bool GMCLSensor::InitSensor(pf_t *pf, GMCLSensorData *data)
{
  return false;
}


////////////////////////////////////////////////////////////////////////////////
// Apply the sensor model
bool GMCLSensor::UpdateSensor(pf_t *pf, GMCLSensorData *data) 
{                                                                       
  return false;
}


#ifdef INCLUDE_RTKGUI

////////////////////////////////////////////////////////////////////////////////
// Setup the GUI
void GMCLSensor::SetupGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Shutdown the GUI
void GMCLSensor::ShutdownGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig)
{
  return;
}


////////////////////////////////////////////////////////////////////////////////
// Draw sensor data
void GMCLSensor::UpdateGUI(rtk_canvas_t *canvas, rtk_fig_t *robot_fig, GMCLSensorData *data)
{
  return;
}

#endif
