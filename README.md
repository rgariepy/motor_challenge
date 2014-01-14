# Motor Challenge

## Goal
Given the provided data, create a node or nodes that detects when one or both encoders are faulty and publishes to the "status" topic.

## Installation, Compilation, and Execution
To use the provided example, try the following instructions:

1. Install ROS Hydro from the instructions at www.ros.org. Ubuntu is highly recommended  
2. Open a terminal and enter the following commands to create a "Catkin" workspace  
> cd ~  
> source /opt/ros/hydro/setup.bash  
> mkdir -p challenge_ws/src && cd challenge_ws  
> catkin_init_workspace src  
> catkin_make  
> echo "source ~/challenge_ws/devel/setup.bash" >> ~/.bashrc  
> . ~/.bashrc  
3. Copy the motor_challenge repository into ~/challenge_ws/src/  
> roscd && cd ..  
> catkin_make  
> catkin_make    
(note that catkin_make was run twice)  
4. At this point, the code should compile. Run it with:  
roslaunch motor_challenge motor_challenge.launch  
