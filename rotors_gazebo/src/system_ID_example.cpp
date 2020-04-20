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

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double desired_yaw = 0.0;

  trajectory_msg.header.stamp = ros::Time::now();
  Eigen::Vector3d desired_position(0.0, 0.0, 10.0);
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(2.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -5.0;
  desired_position[1] = -2.0;
  desired_position[2] = 8.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(2.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -3.0;
  desired_position[1] = 1.0;
  desired_position[2] = 8.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(2.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = +4.0;
  desired_position[1] = -1.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = +1.0;
  desired_position[1] = -1.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 2.5;
  desired_position[1] = -0.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -3.50;
  desired_position[1] = 4.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -3.8;
  desired_position[1] = -4.60;
  desired_position[2] = 4.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -7.0;
  desired_position[1] = -8.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(5.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -6.0;
  desired_position[1] = 0.0;
  desired_position[2] = 5.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(4.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 5.0;
  desired_position[1] = -2.0;
  desired_position[2] = 8.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(2.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 7.0;
  desired_position[1] = 4.0;
  desired_position[2] = 10.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(5.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 4.9;
  desired_position[1] = -4.0;
  desired_position[2] = 13.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 5.9;
  desired_position[1] = -1.5;
  desired_position[2] = 11.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -4.5;
  desired_position[1] = -2.0;
  desired_position[2] = 13.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -3.50;
  desired_position[1] = -4.0;
  desired_position[2] = 13.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -3.8;
  desired_position[1] = 4.60;
  desired_position[2] = 14.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(3).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 7.0;
  desired_position[1] = -8.0;
  desired_position[2] = 12.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(5.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = -6.0;
  desired_position[1] = 10.0;
  desired_position[2] = 10.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(13.0).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 3.50;
  desired_position[1] = -4.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 2.50;
  desired_position[1] = -5.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 3.50;
  desired_position[1] = -4.0;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 5.50;
  desired_position[1] = -4.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();

  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 5.50;
  desired_position[1] = -4.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 6.50;
  desired_position[1] = -5.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 7.50;
  desired_position[1] = -5.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();
  trajectory_msg.header.stamp = ros::Time::now();
  desired_position[0] = 8.50;
  desired_position[1] = -6.5;
  desired_position[2] = 3.0;
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  ros::Duration(0.5).sleep();


  // // Overwrite defaults if set as node parameters.
  // nh_private.param("x", desired_position.x(), desired_position.x());
  // nh_private.param("y", desired_position.y(), desired_position.y());
  // nh_private.param("z", desired_position.z(), desired_position.z());
  // nh_private.param("yaw", desired_yaw, desired_yaw);

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(), desired_position.x(),
  //          desired_position.y(), desired_position.z());

  ros::spinOnce();
  ros::shutdown();

  return 0;
}
