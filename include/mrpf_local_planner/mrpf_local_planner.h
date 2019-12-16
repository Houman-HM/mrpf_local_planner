
#ifndef MRPF_LOCAL_PLANNER_ROS_H_
#define MRPF_LOCAL_PLANNER_ROS_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include "mrpf_local_planner/robot.h"
#include <base_local_planner/goal_functions.h>
#include <time.h>
#include <fstream>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <array>
#include <vector>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

namespace mrpf_local_planner{

  /**
   * @class MRPFPlannerROS
   * @brief Plugin to the ros base_local_planner.
   */

  class MRPFPlannerROS : public nav_core::BaseLocalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      MRPFPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the elastic band local planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      MRPFPlannerROS(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Destructor for the wrapper
       */
      ~MRPFPlannerROS();

      /**
       * @brief Initializes the ros wrapper
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);
      void quaternionToRPY (std::vector<geometry_msgs::PoseStamped> path);
      void yamlReader(std::string pathToFile);

      /**
       * @brief Set the plan that the controller is following; also reset MRPF-planner
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief  Check if the goal pose has been achieved
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:

  //Pointer to external objects (do NOT delete object)
  costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap  
  tf::TransformListener* tf_; ///<@brief pointer to Transform Listener 
  // Data
  bool planReceived_ = false;
  bool velocityExecuted_ = false;
  std::vector<double> yaw_;
  std::vector<double> mainTrajectoryX_;
  std::vector<double> mainTrajectoryY_;
  double dt_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  geometry_msgs::Twist cmd_;
  ros::Publisher cmd_vel_publisher;

  bool goal_reached_;
  bool timeUpdated_;
  bool initialized_;
  int counter_ = 0;
  int max_vel = 0.5;
  bool rotation_; 
  std::chrono::high_resolution_clock::time_point currentTime_;
  std::chrono::high_resolution_clock::time_point previousTime_;
  double timeDifference;                                                                                                                                                                                                                                                                                                                                                              
  std::vector<Robot> robots_;
  double maxVelocity_;
  void setVelZ();
  void publishPath();
  void calculateVelocities();
  void calculateDxAndDy();
  void calculateDistance();
  void transformPoints();
  void velocitiesInRobotFrame();
  double getYaw(geometry_msgs::PoseWithCovarianceStamped msg);

  };
};

#endif

