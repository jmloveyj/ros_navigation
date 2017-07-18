# ros_navigation
robot automatic navigation stack including real-time mapping method(HIMM), obstacle avoidance(vfh+) and navigation algorithms(A*, RRT+)

histogramic in-motion mapping (HIMM) please refer to: HISTOGRAMIC IN-MOTION MAPPING
FOR MOBILE ROBOT OBSTACLE AVOIDANCE

Rapidly-exploring Random Trees (RRTs) please refer to:
RRT-Connect: An Efficient Approach to Single-Query Path Planning 

grid_map pakckage is from https://github.com/ethz-asl/grid_map

 

roslaunch move_control testMap.launch 
You can edit testMap.launch to try mapTest(Navigation of rrt+ and obstacle avoidance), mapTest_vfh(Only obstacle avoidance with moving tiny map to fufill point-point navigation), mapTest_graph(bi-direction graph navigation with A* and vfh for obstacle avoidance)