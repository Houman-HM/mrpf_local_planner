
#include "mrpf_local_planner/mrpf_local_planner.h"
#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>  
// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>
#include "yaml-cpp/yaml.h"

PLUGINLIB_DECLARE_CLASS(mrpf_local_planner, MRPFPlannerROS, mrpf_local_planner::MRPFPlannerROS, nav_core::BaseLocalPlanner)

namespace mrpf_local_planner
{

MRPFPlannerROS::MRPFPlannerROS()
: costmap_ros_(NULL),
  tf_(NULL), 
	initialized_(false) 
{}

MRPFPlannerROS::MRPFPlannerROS(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
: costmap_ros_(NULL), tf_(NULL), initialized_(false)
{
	// initialize planner
	initialize(name, tf, costmap_ros);
}

MRPFPlannerROS::~MRPFPlannerROS() {}


	void MRPFPlannerROS::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
	{

		// check if the plugin is already initialized
		if(!initialized_)
		{

		// copy adress of costmap and Transform Listener (handed over from move_base)
		costmap_ros_ = costmap_ros;
		tf_ = tf;

		initialized_ = true;

		// this is only here to make this process visible in the rxlogger right from the start
		ROS_DEBUG("MRPF Local Planner plugin initialized.");
		}
		else
		{
		ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool MRPFPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{

  // check if plugin initialized
  if(!initialized_)
  {
  ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
  return false;
  }
  //reset next counter
  if (!planReceived_)
  {
    goal_reached_ = false;
    plan_ = orig_global_plan;
    quaternionToRPY(plan_);
    mainTrajectoryX_[0] = 0.0;
    yaw_[0] = 0;
    mainTrajectoryY_[0] = 0.0;
    dt_ = 30.0/mainTrajectoryX_.size();
    yamlReader("/home/houman/catkin_ws/src/new_nodes/params/robots.yaml");
    transformPoints();
    calculateDxAndDy();
    calculateVelocities();
    publishPath();

    planReceived_ = true;
	}
  return true;
  }
	bool MRPFPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
	{
		// check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

    currentTime_ = std::chrono::high_resolution_clock::now();
    if (!timeUpdated_)
    {
      previousTime_ = std::chrono::high_resolution_clock::now();
      timeUpdated_ = true;
    }
    timeDifference = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime_ - previousTime_).count()/1000.0;
    // std::cout << "Current time is: " << currentTime_ <<std::endl;
    // std::cout << "Previous time is: " << previousTime_ << std::endl;
    std::cout << "The time difference is " << timeDifference << "and dt is "<< dt <<std::endl;
		if (timeDifference < dt_)
    {
      for (int j = 0; j < robots_.size(); j++)
      {
        robots_[j].cmd_vel_publisher_.publish(robots_[j].twist);
      }      
    }
    else
    {
      std::cout <<"Sending a new velocity" << std::endl;
      if (counter_ < robots_[0].vx_.size())
      {
        for (int j = 0; j < robots_.size(); j++)
        {
        robots_[j].twist.linear.x = robots_[j].vx_[counter_];
        robots_[j].twist.linear.y = robots_[j].vy_[counter_];
        robots_[j].cmd_vel_publisher_.publish(robots_[j].twist);
      }
      counter_++;
      previousTime_ = std::chrono::high_resolution_clock::now();

    }
    else
    {
      setVelZ();
      goal_reached_ = true;
    }
    }
		return true;
	}

  void MRPFPlannerROS::publishPath()
  { 
    for (int i = 0; i < robots_.size(); i++)
    {
      for(int j = 0; j < mainTrajectoryX_.size(); j++)
      {
        robots_[i].trajectory_pose_.pose.position.x = robots_[i].transformed_x_[j];
        robots_[i].trajectory_pose_.pose.position.y = robots_[i].transformed_y_[j];
        robots_[i].trajectory_pose_.pose.orientation.x = 0;
        robots_[i].trajectory_pose_.pose.orientation.y = 0;
        robots_[i].trajectory_pose_.pose.orientation.z = 0;
        robots_[i].trajectory_pose_.pose.orientation.w = 1;
        robots_[i].trajectory_pose_.header.frame_id = "map";
        robots_[i].trajectory_.header.frame_id = "map";
        robots_[i].trajectory_.poses.push_back(robots_[i].trajectory_pose_);
      }
        robots_[i].path_publisher_.publish(robots_[i].trajectory_);
        ros::Duration(1).sleep();
        robots_[i].path_publisher_.publish(robots_[i].trajectory_);
    }
  }
	bool MRPFPlannerROS::isGoalReached()
	{
		// check if plugin initialized
		if(!initialized_)
		{
			ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
			return false;
		}
		// this info comes from compute velocity commands:
		return goal_reached_;
	}

  void MRPFPlannerROS::yamlReader(std::string pathToFile)
  {
  try
  {
    YAML::Node config = YAML::LoadFile(pathToFile);
    for (int i = 0; i < config["Robots"].size(); i++)
    {
      robots_.push_back(Robot(config["Robots"][i]["name"].as<std::string>(), config["Robots"][i]["vertex"][0].as<double>(),
                        config["Robots"][i]["vertex"][1].as<double>(), config["Robots"][i]["initial_angle"].as<double>(), 
                        config["Robots"][i]["path_topic"].as<std::string>(), 
                        config["Robots"][i]["cmd_vel_topic"].as<std::string>()));
    }
    rotation_ = config["Global"]["rotation"].as<bool>();
      
  }

  catch(...)
  { 
    ROS_INFO("Could not read the Yaml file.");
  }
}

void MRPFPlannerROS::quaternionToRPY (std::vector<geometry_msgs::PoseStamped> path)
{
    
	for (int i = 0; i < path.size(); i++)
	{
		tf::Quaternion q(
		path[i].pose.orientation.x,
		path[i].pose.orientation.y,
		path[i].pose.orientation.z,
		path[i].pose.orientation.w);
		tf::Matrix3x3 m(q);
		double r, p, y;
		m.getRPY(r, p, y);
		double xx= path[i].pose.position.x;
		double yy= path[i].pose.position.y;
		yaw_.push_back(y);
		mainTrajectoryX_.push_back(xx);
		mainTrajectoryY_.push_back(yy);
  }
}

void MRPFPlannerROS::setVelZ()
{

  cmd_.linear.x= 0;
  cmd_.linear.y= 0;
  cmd_.angular.z=0;
  for (int j = 0; j < robots_.size(); j++)
  {
    robots_[j].cmd_vel_publisher_.publish(cmd_);
  }
}


void MRPFPlannerROS::transformPoints()
{
  for (int i = 0; i < plan_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
    robots_[j].transformed_x_.push_back(mainTrajectoryX_[i] + robots_[j].vertex_x_ * cos(yaw_[i]));
    robots_[j].transformed_y_.push_back(mainTrajectoryY_[i] + robots_[j].vertex_x_ * sin(yaw_[i]));
    }
  }
}

void MRPFPlannerROS::calculateVelocities()
{
  for (int i = 0; i < robots_[0].dx_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
    robots_[j].vx_.push_back(robots_[j].dx_[i]/dt_);
    robots_[j].vy_.push_back(robots_[j].dy_[i]/dt_);
    }
  }
  std::cout << "Finished calculating Vx and Vy which were " << robots_[0].vx_.size() << " elements" << std::endl;
}

void MRPFPlannerROS::calculateDxAndDy()
{
  for (int i = 0;  i < robots_.size(); i++)
  {
    for (int j = 0; j < robots_[0].transformed_x_.size()-1; j++)
    {
      robots_[i].dx_.push_back(robots_[i].transformed_x_[j+1] - robots_[i].transformed_x_[j]);
      robots_[i].dy_.push_back(robots_[i].transformed_y_[j+1] - robots_[i].transformed_y_[j]);
    }
  }
  std::cout << "Finished calculating Dx and Dy which were " << robots_[0].dx_.size() << " points"<< std::endl;
}
void MRPFPlannerROS::velocitiesInRobotFrame()
  {
    for (int i = 0; i <robots_.size(); i++)
    {
      for (int j = 0; j <robots_[i].dx_.size(); j++)
      {
        robots_[i].dx_prime_.push_back(robots_[i].dx_[j] * cos((yaw_[j] + robots_[i].initial_angle_ + yaw_[j+1] + robots_[i].initial_angle_)/2)
        + robots_[i].dy_[j] * sin((yaw_[j]+ robots_[i].initial_angle_ + yaw_[j+1]+ robots_[i].initial_angle_)/2));
        robots_[i].dy_prime_.push_back(robots_[i].dy_[j] * cos((yaw_[j]+ robots_[i].initial_angle_ + yaw_[j+1] + robots_[i].initial_angle_)/2) 
        - robots_[i].dx_[j] * sin((yaw_[j] + robots_[i].initial_angle_ + yaw_[j+1] + robots_[i].initial_angle_)/2));
      }
    }
  }

}


