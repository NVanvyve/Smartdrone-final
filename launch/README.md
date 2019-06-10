# Files : 

system.launch : launches mavros and the positioning system containing the total station and the UWB system. 

t265.launch : launches mavros and the camera 

rs_t265_pose_only.launch: launches only the camera (publishes on /camera/odom/sample) 

# Launch the systems : 

`roslaunch smart_drone $FILE_NAME --screen`

NOTE :

* The mavros package needs to be launched for any of the individual ROS nodes to work. This is done in the system.launch file and the t265.launch file. Launching mavros twice will cause problems. Thus be sure, when launching simultaneously multiple launch files containing a mavros launch, to comment the line launching mavros in all files but one. 

* When launching a single file, make sure, a mavros launch is done somewhere. 

* When launching a file and it refuses, try launching the master node first : `roscore` 
