
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
    while (!plan_generated_)
    {
      ROS_INFO("Waiting for the plan!");
    }
    ROS_INFO("New goal has been received!");
    goal_reached_ = false;
    plan_ = orig_global_plan;
    yamlReader("/home/houman/catkin_ws/src/new_nodes/params/robots.yaml");
    yaw_[0] = initial_pose.orientation.z;
    // quaternionToRPY(plan_);
    trajectorySmoothener();
    distanceFromMainTrajectory();
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

  // Smoothening the trajectory by adding waypoints to it.
  void MRPFPlannerROS::trajectorySmoothener()
  {
    int i = 1;
    double difference;
    int num_elements = 0;
    while (i < yaw_.size())
    { 
      difference = yaw_[i] - yaw_[i-1];
      num_elements = int(fabs(difference/0.06));
      if (fabs(difference)>=0.07)
      {
      std::vector<double> temp = linspace(yaw_[i-1], yaw_[i],num_elements);
      yaw_.insert(yaw_.begin()+i, temp.begin(), temp.end());
      temp = linspace(main_trajectory_x_[i-1], main_trajectory_x_[i], num_elements);
      main_trajectory_x_.insert(main_trajectory_x_.begin()+i, temp.begin(), temp.end());
      temp = linspace(main_trajectory_y_[i-1], main_trajectory_y_[i], num_elements);
      main_trajectory_y_.insert(main_trajectory_y_.begin()+i, temp.begin(), temp.end());
      }
      i++;
    }
    std::cout << "Trajectory was smoothened successfully!" << std::endl;
  }

  void MRPFPlannerROS::distanceFromMainTrajectory()
  {
    for (int j = 0; j < robots_.size(); j++)
    {
      robots_[j].distnace_from_main_ = (sqrt(pow(robots_[j].vertex_.x - center_coordinates_.x,2) +
      pow(robots_[j].vertex_.y - center_coordinates_.y,2)));
    }
  }

// Converting the main trajectory to the corresponding trajectory for each robot.
  void MRPFPlannerROS::transformPoints()
  {
    for (int i = 0; i < robots_.size(); i++)
    { 
      robots_[i].angle_to_goal_ = atan2((robots_[i].vertex_.y - center_coordinates_.y), 
      (robots_[i].vertex_.x - center_coordinates_.x)); 
    }
  
  geometry_msgs::Point point;
  for (int i = 0; i < main_trajectory_x_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
      point.x = main_trajectory_x_[i] + robots_[j].distnace_from_main_ * cos(yaw_[i] + robots_[j].angle_to_goal_);
      point.y = main_trajectory_y_[i] + robots_[j].distnace_from_main_ * sin(yaw_[i] + robots_[j].angle_to_goal_);
      robots_[j].transformed_points_.push_back(point);
    }
  }
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
      for (int j = 0; j < robots_.size(); j++)
      {
        if (robots_[j].name_ == "main")
        {
          dyaw_.push_back((yaw_[i+1] - yaw_[i]));
        }
      }      
    }
  std::cout << "Finished calculating Dx and Dy which were " << robots_[0].dx_.size() << " points"<< std::endl;
}

// Calculating the distance between each two consecuitive waypoints in the generated trajectory.
void MRPFPlannerROS::calculateDistance()
{
  for (int i = 0; i < robots_.size(); i++)
  {
    for (int j = 0; j < robots_[0].dx_.size(); j++)
    {
      if (robots_[i].name_ =="main")
      {
        robots_[i].distance_.push_back(sqrt(pow(robots_[i].dx_[j],2) + pow(robots_[i].dy_[j],2)));
         dt_.push_back(robots_[0].distance_[j]/max_velocity_);
      }
      else
      {
        robots_[i].distance_.push_back(sqrt(pow(robots_[i].dx_[j],2) + pow(robots_[i].dy_[j],2)));
      }
    }  
  }
  std::cout << "robots distance has "<<robots_[0].distance_.size() << "elements" <<std::endl;
  std::cout << "robots dt has "<<dt_.size() << " elements" <<std::endl;
}

// Map the velocities from the map frame to the robots' frame.
void MRPFPlannerROS::velocitiesInRobotFrame()
  {
    for (int i = 0; i < robots_.size(); i++)
    {
    std::cout<<robots_[i].name_ << " initial angle is " << robots_[i].initial_angle_ <<std::endl;
    }
    for (int i = 0; i <robots_.size(); i++)
    {
      for (int j = 0; j <robots_[i].dx_.size(); j++)
      {
        robots_[i].dx_prime_.push_back(robots_[i].dx_[j] * cos((yaw_[j] + yaw_[j+1])/2)
        + robots_[i].dy_[j] * sin((yaw_[j] + yaw_[j+1])/2));
        robots_[i].dy_prime_.push_back(robots_[i].dy_[j] * cos((yaw_[j]+ + yaw_[j+1])/2) 
        - robots_[i].dx_[j] * sin((yaw_[j] + yaw_[j+1])/2));
      }
    }
    std::cout << "robots dx_prime has "<<robots_[0].dx_prime_.size() << "elements" <<std::endl;
  }

void MRPFPlannerROS::calculateVelocities()
{
  for (int i = 0; i < robots_[0].dx_.size(); i++)
  {
    for (int j = 0; j < robots_.size(); j++)
    {
      if (rotation_)
      {
        geometry_msgs::Twist temp;
        temp.linear.x = robots_[j].dx_prime_[i]/dt_[i];
        temp.linear.y = robots_[j].dy_prime_[i]/dt_[i];
        temp.angular.z = dyaw_[i]/dt_[i];
        robots_[j].velocity_.push_back(temp);
      }
      
      else
      {
        if (robots_[j].name_=="main")
        {
          geometry_msgs::Twist temp;
          temp.linear.x = robots_[j].dx_prime_[i]/dt_[i];
          temp.linear.y = robots_[j].dy_prime_[i]/dt_[i];
          temp.angular.z = dyaw_[i]/dt_[i];
          robots_[j].velocity_.push_back(temp);
        }
        else
        {
          geometry_msgs::Twist temp;
          temp.linear.x = cos(robots_[j].initial_angle_) * (robots_[j].dx_[i]/dt_[i]) + sin(robots_[j].initial_angle_) * 
          (robots_[j].dy_[i]/dt_[i]);
          temp.linear.y = cos(robots_[j].initial_angle_) * (robots_[j].dy_[i]/dt_[i]) - sin(robots_[j].initial_angle_) * 
          (robots_[j].dx_[i]/dt_[i]);
          robots_[j].velocity_.push_back(temp);
        }
      }
    }
  }
  std::cout << "Finished calculating Vx and Vy which were " << robots_[0].velocity_.size() << " elements" << std::endl;
}

  // A separate thread for publishing the velocities to each robot as well as the base_footprint.
  void MRPFPlannerROS::cmdVelPublisherThread(bool start_thread)
  {
    for (int i = 0; i < robots_[0].velocity_.size(); i++)
    {
        for (int j = 0; j < robots_.size(); j++)
        {
        robots_[j].cmd_vel_publisher_.publish(robots_[j].velocity_[i]);
        }
        ros::Duration(dt_[i]).sleep();
    }
    setVelZ();
    erasePreviousTrajectory();
    goal_reached_ = true;
    ROS_INFO("Goal reached!");
    getRobotPose();
    plan_generated_ = false;
  }

  // Erasing the trajectory after it is executed.
  void MRPFPlannerROS::erasePreviousTrajectory()
  {
    for (int i = 0; i < robots_.size(); i++)
    {
      robots_[i].transformed_points_.clear();
      robots_[i].velocity_.clear();
      robots_[i].distance_.clear();
      robots_[i].dx_.clear();
      robots_[i].dy_.clear();
      robots_[i].dx_prime_.clear();
      robots_[i].dy_prime_.clear();
      main_distance.clear();
      main_trajectory_x_.clear();
      main_trajectory_y_.clear();
      yaw_.clear();
      dyaw_.clear();
      robots_[i].trajectory_.poses.clear();
      dt_.clear();
    }
    std::cout<<"Previous trajectory was cleared!" << std::endl;
  }

  //Publishing each robot's path on the corresponding path topic.
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

  // Getting the plan from the Global planner topic.
  void MRPFPlannerROS::callBack(nav_msgs::Path path)
  {
      if (!plan_generated_)
      { 
        std::cout <<"New plan received!" << std::endl;
        for (int i = 0; i < path.poses.size(); i++)
        { 
          tf::Quaternion q(
          path.poses[i].pose.orientation.x,
          path.poses[i].pose.orientation.y,
          path.poses[i].pose.orientation.z,
          path.poses[i].pose.orientation.w);
          tf::Matrix3x3 m(q);
          double r, p, y;
          m.getRPY(r, p, y);
          double xx= path.poses[i].pose.position.x;
          double yy= path.poses[i].pose.position.y;
          yaw_.push_back(y);
          main_trajectory_x_.push_back(xx);
          main_trajectory_y_.push_back(yy);
        }
      plan_generated_ = true;
      std::cout <<"The main trajectory size in the callback is "<<main_trajectory_x_.size() <<std::endl;
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
          robots_.push_back(Robot(config["Robots"][i]["name"].as<std::string>(), config["Robots"][i]["base_footprint"].as<std::string>(),
                            config["Robots"][i]["vertex"][0].as<double>(),
                            config["Robots"][i]["vertex"][1].as<double>(), config["Robots"][i]["initial_angle"].as<double>(), 
                            config["Robots"][i]["path_topic"].as<std::string>(), 
                            config["Robots"][i]["cmd_vel_topic"].as<std::string>()));
        }
        rotation_ = config["Global"]["rotation"].as<bool>();
        center_coordinates_.x = config["Global"]["center_coordinates"][0].as<double>();
        center_coordinates_.y = config["Global"]["center_coordinates"][1].as<double>();
        max_velocity_ = config["Global"]["max_velocity"].as<double>();
        executed_ = true;   
        ROS_INFO("Successfully read the Yaml file!"); 
      }

      catch(...)
      { 
        ROS_INFO("Could not read the Yaml file.");
      }
    }
  }

  std::vector<double>MRPFPlannerROS::linspace(double start, double end, int num)
  {
    std::vector<double> linspaced;
    double delta = fabs(end - start) / num;
    for(int i=1; i <=num; i++)
      {
        if (end - start < 0)
        {
        linspaced.push_back(start - delta * i);
        }
        else
        {
          linspaced.push_back(start + delta * i);
        } 
      }
    // std::cout << "A vector of size " << linspaced.size() << " was generated!" << std::endl;              
    return linspaced;
  }

  void MRPFPlannerROS::getRobotPose()
  {
    for (int i = 0; i < robots_.size(); i++)
    {
      tf::TransformListener transform_listener_;
      tf::StampedTransform transform_;
      if (robots_[i].name_=="main")
      {
        transform_listener_.waitForTransform("/map", robots_[i].base_footprint_,ros::Time(0), ros::Duration(1.0));
        transform_listener_.lookupTransform("/map", robots_[i].base_footprint_,ros::Time(0), transform_);
        transform_.getBasis().getRPY(initial_pose.orientation.x, initial_pose.orientation.y,
        initial_pose.orientation.z);
        robots_[i].initial_angle_ = initial_pose.orientation.z;
      }
      else
      {
        geometry_msgs::Pose pose;
        transform_listener_.waitForTransform("/map", robots_[i].base_footprint_,ros::Time(0), ros::Duration(1.0));
        transform_listener_.lookupTransform("/map", robots_[i].base_footprint_,ros::Time(0), transform_);
        transform_.getBasis().getRPY(pose.orientation.x, pose.orientation.y,
        pose.orientation.z);
        robots_[i].initial_angle_ = pose.orientation.z;
      }
    }
    ROS_INFO("Robots' poses were updated!");
  }

  // Converting quaternion orinetation of trajectory points to RPY.
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
}