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
 * uos_diffdrive_teleop_key.cpp
 *
 *  Created on: 16.02.2015
 *      Author: Sebastian Pütz <spuetz@uos.de>
 */

#include <uos_diffdrive_teleop_key.h>

int kfd = 0;
struct termios cooked, raw;

TeleopKeyboard::TeleopKeyboard()
:Teleop("uos_diffdrive_teleop_key")
{
  normal_x = this->declare_parameter("normal_x", 0.5);
  normal_y = this->declare_parameter("normal_y", 0.5);
  high_x = this->declare_parameter("high_x", 1.0);
  high_y = this->declare_parameter("high_y", 1.0);
  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  raw.c_cc[VMIN] = 0;
  raw.c_cc[VTIME] = 5;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' to translate");
  puts("Use 'AD' to yaw");
  puts("Use 'QE' to translate and yaw");
  puts("Use 'Space' to break");
  puts("Press 'Shift' to run");
}

void TeleopKeyboard::readKeyboard()
{
  c = 0;
  // get the next event from
  // the keyboard
  if(read(kfd, &c, 1) < 0)
  {
    perror("read():");
    exit(-1);
  }

  in.updated = true;
  
  switch(c)
  {
    // Walking
    case
      KEYCODE_W:
      in.forwards = normal_y;
      break;
    case
      KEYCODE_S:
      in.forwards = -normal_y;
      break;
    case
      KEYCODE_A:
      in.left = normal_x;
      break;
    case
      KEYCODE_D:
      in.left = -normal_x;
      break;
    case
      KEYCODE_Q:
      in.forwards = normal_y;
      in.left = normal_x;
      break;
    case
      KEYCODE_E:
      in.forwards = normal_y;
      in.left = -normal_x;
      break;
    case KEYCODE_SPACE:
      in.forwards = 0;
      in.left = 0;
      break;
    // Running 
    case
      KEYCODE_W_CAP:
      in.forwards = high_y;
      break;
    case
      KEYCODE_S_CAP:
      in.forwards = -high_y;
      break;
    case
      KEYCODE_A_CAP:
      in.left = high_x;
      break;
    case
      KEYCODE_D_CAP:
      in.left = -high_x;
      break;
    case
      KEYCODE_Q_CAP:
      in.forwards = high_y;
      in.left = high_x;
      break;
    case
      KEYCODE_E_CAP:
      in.forwards = high_y;
      in.left = -high_x;
      break;
    default:
      in.forwards *= 0.4;
      in.left *= 0.4;
      in.updated = false;
  }
}

// void quit(int sig)
// {
//   tcsetattr(kfd, TCSANOW, &cooked);
//   exit(0);
// }

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TeleopKeyboard>();

  rclcpp::ExecutorOptions opts;
  rclcpp::executors::MultiThreadedExecutor executor(opts, 2);
  executor.add_node(node);
  
  std::thread executor_thread(
    std::bind(&rclcpp::executors::MultiThreadedExecutor::spin,
            &executor));

  while(rclcpp::ok())
  {
    node->readKeyboard();
  }

  executor_thread.join();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
