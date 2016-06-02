# icub_ros

## ROS and Yarp Packages for iCub HRI Project

#### Following Human Arm in iCub Simulation

Installation:  
`cd ~/catkin_ws/src`   
`git clone https://github.com/tkelestemur/icub_ros.git`   
`cd ~/catkin_ws`   
`catkin_make`

Run in order:  
1. `roscore`
2. `yarp server --ros`  
3. Run simulator: `iCub_SIM`  
4. `simCartesianControl --no_legs`   
5. `iKinCartesianSolver --context simCartesianControl --part right_arm`   
5. `cd ~/catkin_ws/src/icub_ros/iCubSim/build` and `run ./jointFollower`

7. To get data from Kinect V2 Server run:   
`rosrun kinect_client joints --ip 192.168.110.116`
and `rosrun kinect_client tf_publisher`
8. `rosrun icub_ros joint_transformer`
9. or directly run `roslaunch icub_ros joint_follower.launch`
