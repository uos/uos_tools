/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <cstdio>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Services
#include "laser_assembler/AssembleScans.h"

// Messages
#include "sensor_msgs/PointCloud.h"

/***
 * This a simple test app that requests a point cloud from the
 * point_cloud_assembler every 4 seconds, and then publishes the
 * resulting data
 */
namespace laser_assembler
{

class PeriodicSnapshotter
{

public:

  PeriodicSnapshotter()
  {
    // Create a publisher for the clouds that we assemble
    pub_ = n_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

    sub_ = n_.subscribe("joint_states", 1000, &PeriodicSnapshotter::rotCallback, this);

    // Create the service client for calling the assembler
    client_ = n_.serviceClient<AssembleScans>("assemble_scans");

    first_time_ = true;
    arm_ = false;
  }

  /**
   * @return the index of the rotunit joint in the JointState message; -1 if not found
   */
  int getIndex(const sensor_msgs::JointState::ConstPtr& e)
  {
    for (size_t i = 0; i < e->name.size(); ++i)
    {
      if (e->name[i] == "laser_rot_joint")
        return i;
    }
    return -1;
  }

  void rotCallback(const sensor_msgs::JointState::ConstPtr& e)
  {

    if(first_time_) {
      last_time_ = e->header.stamp;
      first_time_ = false;
      return;
    }

    int index = getIndex(e);
    if (index < 0)
      return;

    double position = fmod(e->position[index], 2 * M_PI);

    if (!arm_ && position > 3) {
      arm_ = true;
      return;
    }

    if(arm_ && position > 0 && position < 1) {

      // Populate our service request based on our timer callback times
      AssembleScans srv;
      srv.request.begin = last_time_;
      srv.request.end   = e->header.stamp;

      // Make the service call
      if (client_.call(srv))
      {
        ROS_INFO("Published Cloud with %zu points", srv.response.cloud.points.size()) ;
        pub_.publish(srv.response.cloud);
      }
      else
      {
        ROS_ERROR("Error making service call\n") ;
      }

      arm_ = false;
      last_time_ = e->header.stamp;
    }
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::ServiceClient client_;
  bool first_time_;
  bool arm_;
  ros::Time last_time_;
} ;

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotunit_snapshotter");
  ros::NodeHandle n;
  ROS_INFO("Waiting for [build_cloud] to be advertised");
  ros::service::waitForService("build_cloud");
  ROS_INFO("Found build_cloud! Starting the snapshotter");
  PeriodicSnapshotter snapshotter;
  ros::spin();
  return 0;
}
