
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

		robot1_ = Robot("robot1", -0.6, 0.0, 0.0, "path1", "clearbot1/cmd_vel");
    robot2_ = Robot("robot2", 0.6, 0.0, 0.0, "path2", "clearbot2/cmd_vel");

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
      robot1_.transformed_x_.push_back(main_trajectory_x_[i] + robot1_.vertex_x_ * cos(yaw[i]));
      robot1_.transformed_y_.push_back(main_trajectory_y_[i] + robot1_.vertex_x_ * sin(yaw[i]));
      robot2_.transformed_x_.push_back(main_trajectory_x_[i] + robot2_.vertex_x_ * cos(yaw[i]));
      robot2_.transformed_y_.push_back(main_trajectory_y_[i] + robot2_.vertex_x_ * sin(yaw[i]));
    }


    for (int i = 0; i < main_trajectory_x_.size()-1; i++)
    {
      robot1_.vx_.push_back((robot1_.transformed_x_[i+1]-robot1_.transformed_x_[i])/dt_);
      robot1_.vy_.push_back((robot1_.transformed_y_[i+1]-robot1_.transformed_y_[i])/dt_);
      robot2_.vx_.push_back((robot2_.transformed_x_[i+1]-robot2_.transformed_x_[i])/dt_);
      robot2_.vy_.push_back((robot2_.transformed_y_[i+1]-robot2_.transformed_y_[i])/dt_);

    }
    //     for(int i = 0; i < robot1_.vx_.size(); i++)
    // {
    //   std::cout<< robot1_.vx_[i] << std::endl;
    //   std::cout<< robot1_.vy_[i] << std::endl;
    // }
      
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

			for(int i = 0; i < robot1_.vx_.size(); i++)
			{
				geometry_msgs::Twist r1;
        geometry_msgs::Twist r2;
        r1.linear.x = robot1_.vx_[i];
        r1.linear.y = robot1_.vy_[i];
        r2.linear.x = robot2_.vx_[i];
        r2.linear.y = robot2_.vy_[i];
        robot1_.cmd_vel_publisher_.publish(r1);
        robot2_.cmd_vel_publisher_.publish(r2);
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
    for(int i = 0; i < main_trajectory_x_.size(); i++)
    {
      robot1_.trajectory_pose_.pose.position.x = robot1_.transformed_x_[i];
      robot1_.trajectory_pose_.pose.position.y = robot1_.transformed_y_[i];
      robot1_.trajectory_pose_.pose.orientation.x = 0;
      robot1_.trajectory_pose_.pose.orientation.y = 0;
      robot1_.trajectory_pose_.pose.orientation.z = 0;
      robot1_.trajectory_pose_.pose.orientation.w = 1;
      robot1_.trajectory_pose_.header.frame_id = "map";
      robot1_.trajectory_.header.frame_id = "map";
      robot1_.trajectory_.poses.push_back(robot1_.trajectory_pose_);

      robot2_.trajectory_pose_.pose.position.x = robot2_.transformed_x_[i];
      robot2_.trajectory_pose_.pose.position.y = robot2_.transformed_y_[i];
      robot2_.trajectory_pose_.pose.orientation.x = 0;
      robot2_.trajectory_pose_.pose.orientation.y = 0;
      robot2_.trajectory_pose_.pose.orientation.z = 0;
      robot2_.trajectory_pose_.pose.orientation.w = 1;
      robot2_.trajectory_pose_.header.frame_id = "map";
      robot2_.trajectory_.header.frame_id = "map";
      robot2_.trajectory_.poses.push_back(robot2_.trajectory_pose_);    
    }
    robot1_.path_publisher_.publish(robot1_.trajectory_);
    robot2_.path_publisher_.publish(robot2_.trajectory_);
    ros::Duration(2).sleep();
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



	void MRPFPlannerROS::setVelZ()
	{

		cmd_.linear.x= 0;
		cmd_.linear.y= 0;
		cmd_.angular.z=0;
    robot1_.cmd_vel_publisher_.publish(cmd_);
    robot2_.cmd_vel_publisher_.publish(cmd_);
	}

}


