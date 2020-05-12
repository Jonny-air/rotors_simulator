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
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
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
  ros::Duration(3.0).sleep();

  mav_msgs::RollPitchYawrateThrust mav_msg;
  mav_msg.header.stamp = ros::Time::now();

  // Default desired input
  mav_msg.roll = 0.0;
  mav_msg.pitch = 0.0;
  mav_msg.yaw_rate = 0.0;
  mav_msg.thrust.x = 0.0;
  mav_msg.thrust.y = 0.0;
  // mav_msg.thrust.z = 46.5; //peregrine
  // ROS_INFO("Publishing up");
  // mav_pub.publish(mav_msg);

  // ros::Duration(8.0).sleep();
  // mav_msg.thrust.z = 45.0; //peregrine
  // ROS_INFO("Publishing stop up");
  // mav_pub.publish(mav_msg);

  mav_msg.thrust.z = 70; //peregrine
  // mav_msg.thrust.z = 15.1413010196; //firefly
  // ROS_INFO("Publishing hovering");
  // mav_msg.header.stamp = ros::Time::now();
  // mav_pub.publish(mav_msg);
  // ros::Duration(0.3).sleep();
  mav_msg.roll = 0.63;
  mav_msg.pitch = -0.63;
  for (int i = 0; i <300; i += 1){
    mav_msg.header.stamp = ros::Time::now();
    mav_pub.publish(mav_msg);
    ros::Duration(0.01).sleep();
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  // ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  // Default desired position and yaw.
  Eigen::Vector3d desired_position(0.0, 0.0, 15.0);
  double desired_yaw = 0.0;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());
  nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
