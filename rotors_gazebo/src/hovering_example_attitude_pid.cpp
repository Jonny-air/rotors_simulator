/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  ros::Publisher mav_pub =
      nh.advertise<mav_msgs::RollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  mav_msgs::RollPitchYawrateThrust mav_msg;
  mav_msg.header.stamp = ros::Time::now();

  // Default desired input
  mav_msg.roll = 0.0;
  mav_msg.pitch = 0.0;
  mav_msg.yaw_rate = 0.0;
  mav_msg.thrust.x = 0.0;
  mav_msg.thrust.y = 0.0;
  mav_msg.thrust.z = 46.5;
  ROS_INFO("Publishing up");
  mav_pub.publish(mav_msg);

  ros::Duration(5.0).sleep();
  mav_msg.thrust.z = 45.0;
  ROS_INFO("Publishing stop up");
  mav_pub.publish(mav_msg);

  ros::Duration(3.75).sleep();
  mav_msg.thrust.z = 45.8447083403;
  ROS_INFO("Publishing hovering");
  mav_pub.publish(mav_msg);

  ros::Duration(2).sleep();
  mav_msg.roll = 0.1;
  ROS_INFO("Publishing roll");
  mav_pub.publish(mav_msg);

  // ros::Duration(2).sleep();
  // mav_msg.roll = 0.0;
  // ROS_INFO("Publishing roll");
  // mav_pub.publish(mav_msg);

  // ros::Duration(5).sleep();
  // mav_msg.pitch = 0.1;
  // ROS_INFO("Publishing pitch");
  // mav_pub.publish(mav_msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
