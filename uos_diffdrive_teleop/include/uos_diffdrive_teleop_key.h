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
 * uos_diffdrive_teleop_key.h
 *
 *  Created on: 16.02.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */

#ifndef UOS_DIFFDRIVE_TELEOP_KEY_H
#define UOS_DIFFDRIVE_TELEOP_KEY_H

#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <uos_diffdrive_teleop.h>
#include <sensor_msgs/msg/joy.hpp>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

#define KEYCODE_SPACE 0x20

class TeleopKeyboard 
: public Teleop
{
  public:
    TeleopKeyboard();
    void readKeyboard();

  private:
    char c;    
    double normal_x;
    double normal_y;
    double high_x;
    double high_y;

};
#endif /* uos_diffdrive_teleop_key.h */

