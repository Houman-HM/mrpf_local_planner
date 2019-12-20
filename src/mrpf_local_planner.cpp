
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
    ROS_INFO("New goal has been received!");
    goal_reached_ = false;
    plan_ = orig_global_plan;
    main_trajectory_x_.clear();
    main_trajectory_y_.clear();
    max_velocity_ = 0.1;
    yamlReader("/home/houman/catkin_ws/src/new_nodes/params/robots.yaml");
    erasePreviousTrajectory();
    quaternionToRPY(plan_);
    // yaw_[0] = 0.0;
    transformPoints();
    calculateDxAndDy();
    calculateDistance();
    velocitiesInRobotFrame();
    calculateVelocities();
    publishPath();
    cmdVelPublisherThread(true);
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

    // current_time_ = std::chrono::high_resolution_clock::now();
    // if (!time_updated_)
    // {
    //   previous_time_ = std::chrono::high_resolution_clock::now();
    //   time_updated_ = true;
    // }
    // time_difference_ = std::chrono::duration_cast<std::chrono::milliseconds>(current_time_ - previous_time_).count()/1000.0;
    // // std::cout << "Current time is: " << current_time_ <<std::endl;
    // // std::cout << "Previous time is: " << previous_time_ << std::endl;
    // std::cout << "The time difference is " << time_difference_ << "and dt is "<< dt_ <<std::endl;
		// if (time_difference_ < dt_)
    // {
    //   for (int j = 0; j < robots_.size(); j++)
    //   {
    //     robots_[j].cmd_vel_publisher_.publish(robots_[j].twist);
    //   }      
    // }
    // else
    // {
    //   std::cout <<"Sending a new velocity" << std::endl;
    //   if (counter_ < yaw_.size() -1)
    //   {
    //     for (int j = 0; j < robots_.size(); j++)
    //     {
    //     dt_ = robots_[0].distance_[j]/max_velocity_;
    //     robots_[j].twist.linear.x = max_velocity_ * cos(yaw_[counter_]);
    //     robots_[j].twist.linear.y = max_velocity_ * sin(yaw_[counter_]);
    //     robots_[j].cmd_vel_publisher_.publish(robots_[j].twist);
    //   }
    //   counter_++;
    //   previous_time_ = std::chrono::high_resolution_clock::now();

    // }
    // else
    // {
    //   setVelZ();
    //   goal_reached_ = true;
    // }
    // }
		return true;
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
		main_trajectory_x_.push_back(xx);
		main_trajectory_y_.push_back(yy);
  }
  std::cout<< "Size of the trajectory is "<<main_trajectory_y_.size() << std::endl;
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
  geometry_msgs::Point point;
  for (int i = 0; i < main_trajectory_x_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
      point.x = main_trajectory_x_[i] + robots_[j].vertex_x_ * cos(yaw_[i]);
      point.y = main_trajectory_y_[i] + robots_[j].vertex_x_ * sin(yaw_[i]);
      robots_[j].transformed_points_.push_back(point);
    }
  }

  // for (int i = 0; i < main_trajectory_x_.size(); i++)
  // {
  //   main_trajectory_x_[i] = main_trajectory_x_[i] * cos(yaw_[i]);
  //   main_trajectory_y_[i] = main_trajectory_y_[i] * sin(yaw_[i]);
  // }  
}

void MRPFPlannerROS::calculateDxAndDy()
{
  for (int i = 0;  i < robots_.size(); i++)
  {
    for (int j = 0; j < robots_[0].transformed_points_.size()-1; j++)
    {
      robots_[i].dx_.push_back(robots_[i].transformed_points_[j+1].x - robots_[i].transformed_points_[j].x);
      robots_[i].dy_.push_back(robots_[i].transformed_points_[j+1].y - robots_[i].transformed_points_[j].y);
    }
  }
    for (int i = 0; i < main_trajectory_x_.size()-1; i++)
    {
      // main_dx_.push_back(main_trajectory_x_[i+1] - main_trajectory_x_[i]);
      // main_dy_.push_back(main_trajectory_y_[i+1] - main_trajectory_y_[i]);
      dyaw_.push_back(yaw_[i+1] - yaw_[i]);
    }
  std::cout << "Finished calculating Dx and Dy which were " << robots_[0].dx_.size() << " points"<< std::endl;
}

void MRPFPlannerROS::calculateDistance()
{
  for (int i = 0; i < robots_.size(); i++)
  {
    for (int j = 0; j < robots_[0].dx_.size(); j++)
    {
      robots_[i].distance_.push_back(sqrt(pow(robots_[i].dx_[j],2) + pow(robots_[i].dy_[j],2)));
    }  
  }
    for (int i = 0; i < robots_[0].dx_.size(); i++)
    {
      // main_distance.push_back(sqrt(pow(main_dx_[i],2) + pow(main_dy_[i],2)));
      dt_.push_back(robots_[0].distance_[i]/max_velocity_);
    }
}
void MRPFPlannerROS::calculateVelocities()
{
  geometry_msgs::Twist temp;
  for (int i = 0; i < robots_[0].dx_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
    temp.linear.x = robots_[j].dx_prime_[i]/dt_[i];
    temp.linear.y = robots_[j].dy_prime_[i]/dt_[i];
    temp.angular.z = dyaw_[i]/dt_[i];
    robots_[j].velocity_.push_back(temp);
    }
    // temp.linear.x = main_dx_[i]/dt_[i];
    // temp.linear.y = main_dy_[i]/dt_[i];
    // temp.angular.z = dyaw_[i]/dt_[i];
    // main_velocities_.push_back(temp);
  }
  std::cout << "Finished calculating Vx and Vy which were " << robots_[0].velocity_.size() << " elements" << std::endl;
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

  void MRPFPlannerROS::cmdVelPublisherThread(bool start_thread)
  {
    setVelZ();
    for (int i = 0; i < robots_[0].velocity_.size(); i++)
    {
        for (int j = 0; j < robots_.size(); j++)
        {
        robots_[j].cmd_vel_publisher_.publish(robots_[j].velocity_[i]);
        }
        // cmd_vel_publisher.publish(robots_[2].velocity_[i]);
        ros::Duration(dt_[i]).sleep();
        std::cout << "This is the counter: " << dt_[i] <<std::endl;
    }
    setVelZ();
    goal_reached_ = true;
    ROS_INFO("Goal reached!");
  }
  void MRPFPlannerROS::erasePreviousTrajectory()
  {
    for (int i = 0; i < robots_.size(); i++)
    {
      robots_[i].transformed_points_.clear();
      robots_[i].transformed_points_.clear();
      robots_[i].velocity_.clear();
      robots_[i].distance_.clear();
      robots_[i].dx_.clear();
      robots_[i].dy_.clear();
      main_distance.clear();
      main_dx_.clear();
      main_dy_.clear();
      robots_[i].trajectory_.poses.clear();
    }
  }

  void MRPFPlannerROS::publishPath()
  { 
    for (int i = 0; i < robots_.size(); i++)
    {
      for(int j = 0; j < main_trajectory_x_.size(); j++)
      {
        robots_[i].trajectory_pose_.pose.position.x = robots_[i].transformed_points_[j].x;
        robots_[i].trajectory_pose_.pose.position.y = robots_[i].transformed_points_[j].y;
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

  void MRPFPlannerROS::yamlReader(std::string pathToFile)
  {
    if (!executed_)
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
        executed_ = true;   
        ROS_INFO("Successfully read the yaml file!"); 
      }

      catch(...)
      { 
        ROS_INFO("Could not read the Yaml file.");
      }
    }
  }
}