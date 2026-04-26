# robot_welding
download all the requirements and libraries

terminal 1:
cd ws
colcon build
source install/setup.py

ros2 launch brick_wall display.launch.py enable_gap_detector:=true


terminal 2:
cd ws
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

terminal 3:
cd ws
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map

now open rviz :
    change the fixed frmae as map from world
     next in robot model description topic selct /robot_description
      next add robotmodel and select description topic as /wall_description
       next again add robot model amd select description topic as /gap_type

terminal 4 :

ros2 run brick_wall auto_navigator


now u can see robot moving



