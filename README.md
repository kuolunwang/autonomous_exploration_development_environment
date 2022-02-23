# How to run this repo with our robot (Husky) in gazebo

`roslaunch vehicle_simulator husky_tare.launch cmd_velTopic:=/robot/husky_velocity_controller/cmd_vel posestamp_topic:=/slam_pose point_topic:=/map_part`

* input topic
    * /map_part (sensor_msgs/PointCloud2)
        * lidar point with frame_id "map"
    * /slam_pose (geometry_msgs/PoseStamped)
        * robot's PoseStamped with frame_id "map"
    * /move_base_simple/goal (geometry_msgs/PoseStamped)
        * goal's PoseStamped with frame_id "map", you could click it on rviz

* output topic
    * cmd_vel (geometry_msgs/Twist)




<img src="img/header.jpg" alt="Header" width="100%"/>

The repository is meant for leveraging system development and robot deployment for ground-based autonomous navigation and exploration. Containing a variety of simulation environments, autonomous navigation modules such as collision avoidance, terrain traversability analysis, waypoint following, etc, and a set of visualization tools, users can develop autonomous navigation systems and later on port those systems onto real robots for deployment.

Please use instructions on our [project page](https://www.cmu-exploration.com).
