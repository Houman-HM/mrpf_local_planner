# mrpf_local_planner
The local planner developed extends the capabilities of ROS Navigation to handle multi-robot scenarios. The planner considers the robot fleet as a coupled system
## Setup
Clone the respository into to your catkin_ws and build it.
After building, the local planner used by move_base should be set MRPFLocalPlannerROS.

## Details
The local planner features a robots_configuration YAML file in which the parameters related to each robot in a robot fleet should be set.
One should set in the robots_configuration YAML the velocity topic and base_footprint parameter for each robot. In addition, the pose of each robot in the base footprint should be mentioned.
