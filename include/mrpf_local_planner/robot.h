#ifndef MRPF_ROBOT_H_
#define MRPF_ROBOT_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Header.h>

class Robot
{  
public: 
  std::string name_;
  geometry_msgs::Point vertex_;
  bool is_in_back = false;
  double initial_angle_;
  double distnace_from_main_;
  std::vector<geometry_msgs::Point> transformed_points_;
  std::vector<double> dx_;
  std::vector<double> dy_;
  std::vector<double> dtheta_;
  std::vector <double> distance_;
  std::vector<double> dx_prime_;
  std::vector<double> dy_prime_;
  std::vector<geometry_msgs::Twist> velocity_;
  geometry_msgs::PoseStamped trajectory_pose_;
  std_msgs::Header header_;
  nav_msgs::Path trajectory_;
  ros::Publisher path_publisher_;
  ros::Publisher cmd_vel_publisher_;

  Robot(std::string name, double vertex_x, double vertex_y, double initial_angle, std::string path_topic, std::string cmd_vel_topic)
  {
    name_ = name;
    vertex_.x = vertex_x;
    vertex_.y = vertex_y;
    initial_angle_= initial_angle;
    path_publisher_ = n.advertise<nav_msgs::Path>(path_topic, 1000);
    cmd_vel_publisher_ = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1000);
  }
  Robot(){};

private:
  ros::NodeHandle n;

};


#endif