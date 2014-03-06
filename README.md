# Motor Challenge

## Goal
Given the provided data, create a node or nodes that detects when one or both encoders are faulty. Publish "false" to the "status" topic when a fault is present.

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

## Provided data and parameters
Some parameters of the "robot" used to generate the data can be found in the motor_challenge.launch file.  
A bag file containing sample data (with encoder faults) can be found in motor_challenge/bags/ and can be played back using the "rosbag" utility.  

## Testing
The node is to be tested for:
+ Responce time
+ Robustness, e.g. noise tolerance
The tests can be peformed using a model of robot with simulated faults. Difference between start of fault simulation and actual detection provide response time.  Robustenss is tested by using different noise level in measuremnts   with comparing allowed probabilty of the false positive detection.
