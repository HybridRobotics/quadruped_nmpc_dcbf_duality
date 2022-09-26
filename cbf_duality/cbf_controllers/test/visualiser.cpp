//
// Created by qiayuan on 2022/9/12.
//
#include <ros/ros.h>
#include <ocs2_msgs/mpc_observation.h>
#include <visualization_msgs/Marker.h>

ocs2_msgs::mpc_observation MSG_LAST;
size_t ID;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "visualiser");
  ros::NodeHandle nh;

  auto pub_marker = nh.advertise<visualization_msgs::Marker>("/visualiser", 10);

  auto callback = [&](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    if (MSG_LAST.state.value.empty())
    {
      MSG_LAST = *msg;
      ID = 0;
      return;
    }
    if ((msg->time - MSG_LAST.time) < 0.1)
      return;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "odom";
    marker.id = ++ID;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.015;
    marker.color.r = 0.4660;
    marker.color.g = 0.6740;
    marker.color.b = 0.1880;
    marker.color.a = 1.0;
    geometry_msgs::Point point;
    point.z = 0.3;
    point.x = MSG_LAST.state.value[6];
    point.y = MSG_LAST.state.value[7];
    marker.points.push_back(point);
    point.x = msg->state.value[6];
    point.y = msg->state.value[7];
    marker.points.push_back(point);
    MSG_LAST = *msg;
    pub_marker.publish(marker);
  };
  auto sub = nh.subscribe<ocs2_msgs::mpc_observation>("/legged_robot_mpc_observation", 10, callback);

  ros::spin();
  return 0;
}
