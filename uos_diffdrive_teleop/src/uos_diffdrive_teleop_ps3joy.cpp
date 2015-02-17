/*
 *
 * Copyright (C) 2015 University of Osnabrück, Germany
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * uos_diffdrive_teleop_ps3.cpp
 *
 *  Created on: 16.02.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */

#include <uos_diffdrive_teleop_ps3joy.h>

TeleopPS3::TeleopPS3(){
  ros::NodeHandle n_private("~");
  //enable button pressure intensity
  n_private.param("use_button_pressure", use_button_pressure, false);
  joy_sub = n_.subscribe<sensor_msgs::Joy>("joy", 15,  &TeleopPS3::PS3Callback, this);
}

void TeleopPS3::PS3Callback(const sensor_msgs::Joy::ConstPtr& joy) {

  // factor elem of [0 1]
  double factor_x = 0;
  double factor_y = 0;
  double fac_button_x = 0;
  double fac_button_y = 0;

  // read the stick intensity factor
  double fac_stick_y = joy->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
  double fac_stick_x = joy->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];

  // if required, read the button pressure as intensity factor
  if(use_button_pressure) {
    // left, normalize scale  
    double fac_button_up = (1-joy->axes[PS3_AXIS_BUTTON_CROSS_UP])/2.0;
    double fac_button_down = -(1-joy->axes[PS3_AXIS_BUTTON_CROSS_DOWN])/2.0;
    fac_button_y = fac_button_up + fac_button_down;
    // right, normalize scale
    double fac_button_left = (1-joy->axes[PS3_AXIS_BUTTON_CROSS_LEFT])/2.0;
    double fac_button_right = -(1-joy->axes[PS3_AXIS_BUTTON_CROSS_RIGHT])/2.0;
    fac_button_x = fac_button_left + fac_button_right;
  } else { 
    // no button pressure required, check if the factor should be 1 or 0
    // if there is no button pressure enabled the factor should be 1
    fac_button_y = joy->buttons[PS3_BUTTON_CROSS_UP] - joy->buttons[PS3_BUTTON_CROSS_DOWN];
    fac_button_x = joy->buttons[PS3_BUTTON_CROSS_LEFT] - joy->buttons[PS3_BUTTON_CROSS_RIGHT]; 
  }
  // enable both, stick and buttons at the same time
  factor_x = fac_button_x + fac_stick_x; // can be > 1 or < -1
  factor_y = fac_button_y + fac_stick_y; // can be > 1 or < -1

  in.forwards = factor_y;
  in.left = factor_x;
  in.updated = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "uos_diffdrive_teleop_ps3");
  TeleopPS3 teleop;
  ros::spin();
  return EXIT_SUCCESS;
}
