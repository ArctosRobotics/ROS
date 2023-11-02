# Arctos Robotic Arm (ROS Packages)

This repository contains ROS packages for the Arctos robotic arm, enabling motion planning, execution, and simulation in both virtual and real environments.
initially forked from [jesseweisberg/moveo_ros](https://github.com/jesseweisberg/moveo_ros) and edited match Arctos 6 axis robotic arm. 
## How to Use:

### Setting Up Arctos Simulation with Motion Planning
![moveit_screenshot.jpg](/moveit_screenshot.jpg)

1. Make sure you have ROS properly installed with a working workspace. This repository assumes ROS Melodic on Ubuntu 18.04, so make any necessary adjustments if you are using a different configuration. Place the 'arctos_ros' package in the 'src' directory of your catkin workspace.

2. To plan and execute trajectories for the Arctos arm in simulation (using RVIZ with Moveit plugin), run the following terminal command:
   ```
   roslaunch arctos_config demo.launch
   ```

3. Once the window loads, enable "Allow Approximate IK Solutions" in the bottom-left corner. Navigate to the "Planning" tab in the Motion Planning panel of RVIZ. You can set a new goal state by either dragging the interactive marker (the light blue ball at the end effector) or selecting one under "Select Goal State." After updating the goal state, clicking "Plan and Execute" will generate and execute the trajectory from the start state to the updated goal state.

### Controlling the Real Robot, Synchronized with the Simulated Arm's Trajectories
4. Make sure to download the AccelStepper library ([AccelStepper Library Download](http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.57.zip)) and the ros_lib library ([rosserial-arduino tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)) in your Arduino development environment.

   - If you already have the ros_lib library in your Arduino libraries directory (<Arduino sketchbook>/libraries), follow the last troubleshooting tip to avoid errors related to "ArmJointState.h". ROS requires you to remove ros_lib and regenerate it every time you introduce a new custom message.

5. Update the pin layout to match your robot and the CNC shield boards in the **'arctos_moveit_arduino.ino'** file and upload it to your Arduino (assuming you are using a MEGA 2560). Ensure that both the real robot and the simulation start in the same position. To set the simulation upright initially, select "home" from the "Select Goal States" in RVIZ.

6. In 'arctos_moveit_convert.cpp', replace the `stepsPerRevolution` array with the steps per revolution (or microsteps per revolution) of each of your motors. If you don't know these values, you can experimentally determine how many microsteps per revolution your motors have using the MultiStepperTest.ino and recording/observing the results.

7. With the simulation already running, execute the following commands in separate terminal windows:

   - ```rosrun rosserial_python serial_node.py /dev/ttyUSB0``` (establishes a rosserial node to communicate with the Arduino).
   - ```rosrun moveo_moveit moveo_moveit_convert``` (converts joint_state rotations from the simulation to steps and publishes them on the /joint_steps topic, which the Arduino script subscribes to).
   - ```rostopic pub gripper_angle std_msgs/UInt16 <angle 0-180>``` (publishes the gripper angle).

**Now, any trajectories planned and executed in the simulation will be echoed on the real Arctos robot.**

## Directory Structure

### arctos_urdf
This directory contains the URDF (Unified Robot Description File) for the Arctos robotic arm. It's essential for simulating the Arctos arm in RVIZ and configuring it with Moveit.

### arctos_moveit_config
Here, you'll find the configuration files for Moveit, a motion planning framework with a plugin for RVIZ. This configuration is designed for use with the Arctos arm.

### arctos_moveit
- _arctos_moveit_convert.cpp_: This node converts joint_state rotations from the simulation (via the 'move_group/fake_controller_joint_states' topic) into steps. It then publishes these steps on the /joint_steps topic. The joint_steps topic is an array of 6 Int16 values (though in the case of the Arctos arm, there are only 5 joints). This array represents the accumulated steps executed by each joint since the arctos_moveit_convert node started running.

- _arctos_move_group_interface_coor_1.cpp_: This node allows you to hardcode a pose/position for the end effector in the script and plan/execute a trajectory to reach that position. It can also read and output the current pose/position of the end effector.

## Troubleshooting
- After following step 7, you should see three new topics created:
  - **/joint_steps**: This topic contains the steps necessary to move each motor to the desired position.
  - **/joint_steps_feedback**: Similar to /joint_steps, but information is published back by the Arduino to verify that it is receiving data correctly.
  - **/gripper_angle**: This topic provides the current angle of the gripper.

- To move the Arctos arm from the command line, use the following command:
  - ```rostopic pub joint_steps arctos_moveit/ArmJointState <Joint1 Joint2 Joint3 Joint4 Joint5 0>```  
  - Replace "Joint1, Joint2, etc." with the number of steps you want each joint to move.

- Use ```rostopic

 list``` and search for these topics to check if they are currently running.

- Use ```rostopic echo /<topic>``` to view the data on \<topic> in your terminal.

- If you encounter the error message: "error: arctos_moveit/ArmJointState.h: No such file or directory," perform the following steps in the terminal:
  ```
  cd <Arduino sketchbook>/libraries
  rm -rf ros_lib 
  rosrun rosserial_arduino make_libraries.py .
  ```
  - More information can be found on the ROS wiki: 
    - In Section 2.2 here: (http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
    - (http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages)
