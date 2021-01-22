# RoboND_Project2_GoChaseIt
Udacity Robotics Software Engineer NanoDegree Program

## Project Description  
In this project, I created two ROS packages inside my `catkin_ws/src`:  
1. `my_robot`:  
holds a robot designed with the Unified Robot Description Format, a white-colored ball, and a gazebo world.  
2. `ball_chaser`:  
contains two C++ ROS nodes (`drive_bot` & `process_image`) to interact with the robot and make it chase the white ball.  
    - `drive_bot`:  
    provides a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities.  
    The service publishes to the wheel joints and return back the requested velocities.  
    - `process_image`  
    reads my robot’s camera image, analyzes it to determine the presence and position of a white ball.  
    If a white ball exists in the image, my node requests a service via a client to drive the robot towards it.  
Project Description
Directory Structure

.Go-Chase-It                                   
# Go Chase It Project
├── my_robot                       # my_robot package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── robot_description.launch
│   │   ├── world.launch
│   ├── meshes                     # meshes folder for sensors
│   │   ├── hokuyo.dae
│   ├── urdf                       # urdf folder for xarco files
│   │   ├── my_robot.gazebo
│   │   ├── my_robot.xacro
│   ├── worlds                      # world folder for world files
│   │   ├── robot.world
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info
├── ball_chaser                    # ball_chaser package                   
│   ├── launch                     # launch folder for launch files   
│   │   ├── ball_chaser.launch
│   ├── src                        # source folder for C++ scripts
│   │   ├── drive_bot.cpp
│   │   ├── process_images.cpp
│   ├── srv                        # service folder for ROS services
│   │   ├── DriveToTarget.srv
│   ├── CMakeLists.txt             # compiler instructions
│   ├── package.xml                # package info                  
└──                              
## Step to test `process_image`

* Clone this repository
* Open the repository and make
cd /home/workspace/RoboND-Term1-P2-Go-Chase-It/catkin_ws/
catkin_make
* Launch my_robot/my_gokart in Gazebo to load both the world and plugins
roslaunch my_robot world.launch
or

roslaunch my_gokart world.launch
* Launch ball_chaser and process_image nodes
cd /home/workspace/RoboND-Term1-P2-Go-Chase-It/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
* Visualize
cd /home/workspace/RoboND-Term1-P2-Go-Chase-It/catkin_ws/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view
 
Now place the white ball at different positions in front of the robot and see if the robot is capable of chasing the ball.

 
## References  
1. [The inertia matrix explained](http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/): GAZEBO ANSWERS  
2. [[ROS Projects] – Exploring ROS using a 2 Wheeled Robot](http://www.theconstructsim.com/ros-projects-exploring-ros-using-2-wheeled-robot-part-1/): The Construct  
3. [uos_tools/uos_common_urdf/common.xacro](https://github.com/uos/uos_tools/blob/fuerte/uos_common_urdf/common.xacro): GitHub
4. [List of moments of inertia](https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors): Wikipedia
5. 机器人操作系统（ROS）浅析  [美] Jason M. O'Kane 著 肖军浩 译
  
