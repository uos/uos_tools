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
 * uos_diffdrive_teleop.h
 *
 *  Created on: 16.02.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */
#ifndef UOS_DIFFDRIVE_TELEOP_H
#define UOS_DIFFDRIVE_TELEOP_H



#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


#define EPSILON_VELO 1e-3

class Teleop : public rclcpp::Node
{
  using Base = rclcpp::Node;
  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    rclcpp::TimerBase::SharedPtr vel_timer;
    rclcpp::TimerBase::SharedPtr key_timer;
  
    // actually its a duration
    double update_inputs_rate;
    double update_velocity_rate;

    void updateVelocity();
    void updateInputs();

    // implement this!

    double max_vel;
    double max_rot_vel;
    
    geometry_msgs::msg::Twist vel_cmd;

    struct acceleration{
      double pos;
      double neg;
      double stop;
    };
    acceleration acc_y;
    acceleration acc_x;

    struct velocity{
      double x;
      double y;
      double dyn_limit_x;
      double dyn_limit_y;
    };
    velocity velo;

    
    // inputs elem of [0 1]
    struct inputs{
      bool updated;
      double forwards;
      double left;
    };

    double adaptVelocity( 
        double time_delta,
        double velocity,
        double factor,
        double acc_stop,
        double acc_neg,
        double acc_pos);

  protected:
    inputs in;
    
  public:
    Teleop(std::string node_name);
};

#endif /* uos_diffdrive_teleop.h */
