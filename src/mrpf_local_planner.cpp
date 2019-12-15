
#include "mrpf_local_planner/mrpf_local_planner.h"
#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>  
// pluginlib macros (defines, ...)
#include <pluginlib/class_list_macros.h>

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
		yaw.push_back(y);
		main_trajectory_x_.push_back(xx);
		main_trajectory_y_.push_back(yy);
  }
}
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
  if (!plan_received_)
  {
    plan_ = orig_global_plan;
    quaternionToRPY(plan_);

    main_trajectory_x_[0] = 0.0;
    yaw[0] = 0;
    main_trajectory_y_[0] = 0.0;
    dt_ = 20.0/main_trajectory_x_.size();
    for (int i = 0; i < plan_.size(); i++)
    {
      for (int j = 0; j < robots_.size(); j++)
      {
      robots_[j].transformed_x_.push_back(main_trajectory_x_[i] + robots_[j].vertex_x_ * cos(yaw[i]));
      robots_[j].transformed_y_.push_back(main_trajectory_y_[i] + robots_[j].vertex_x_ * sin(yaw[i]));
      }
    }


    for (int i = 0; i < main_trajectory_x_.size()-1; i++)
    {
      for (int j = 0; j < robots_.size(); j++)
      {
      robots_[j].vx_.push_back((robots_[j].transformed_x_[i+1]-robots_[j].transformed_x_[i])/dt_);
      robots_[j].vy_.push_back((robots_[j].transformed_y_[i+1]-robots_[j].transformed_y_[i])/dt_);
      }
    }
      
    plan_received_ = true;
  }

		goal_reached_ = false;

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

		if (!velocity_executed_)
		{
      publishPath();

			for(int i = 0; i < robots_[0].vx_.size(); i++)
			{

        ros::Duration(dt_).sleep();
			}

			velocity_executed_ = true;
      setVelZ();
		}
    else
    {
      goal_reached_ = true;
    }
    

		return true;

	}

  void MRPFPlannerROS::publishPath()
  { 
    for (int i = 0; i < robots_.size(); i++)
    {
      for(int j = 0; j < main_trajectory_x_.size(); j++)
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
    }
    ros::Duration(1.0).sleep();
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

}


