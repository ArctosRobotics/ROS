/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * 6-axis robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation on the PC and sent to Arduino via rosserial. It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached.
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <moveo_moveit/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

// Define pin assignments based on cpu_map.h
// Adjust these according to your specific configuration

// Joint 1
#define JOINT1_STEP_PIN        0  // Update with the correct pin for the 6th axis
#define JOINT1_DIR_PIN         1  // Update with the correct pin for the 6th axis
#define JOINT1_ENABLE_PIN      2  // Update with the correct pin for the 6th axis

// Joint 2
#define JOINT2_STEP_PIN        6  // Update with the correct pin for the 6th axis
#define JOINT2_DIR_PIN         7  // Update with the correct pin for the 6th axis
#define JOINT2_ENABLE_PIN      8  // Update with the correct pin for the 6th axis

// Joint 3
#define JOINT3_STEP_PIN        46  // Update with the correct pin for the 6th axis
#define JOINT3_DIR_PIN         48  // Update with the correct pin for the 6th axis
#define JOINT3_ENABLE_PIN      8  // Update with the correct pin for the 6th axis

// Joint 4
#define JOINT4_STEP_PIN        26  // Update with the correct pin for the 6th axis
#define JOINT4_DIR_PIN         28  // Update with the correct pin for the 6th axis
#define JOINT4_ENABLE_PIN      24  // Update with the correct pin for the 6th axis

// Joint 5
#define JOINT5_STEP_PIN        36  // Update with the correct pin for the 6th axis
#define JOINT5_DIR_PIN         34  // Update with the correct pin for the 6th axis
#define JOINT5_ENABLE_PIN      30  // Update with the correct pin for the 6th axis

// Joint 6
#define JOINT6_STEP_PIN        49  // Update with the correct pin for the 6th axis
#define JOINT6_DIR_PIN         51  // Update with the correct pin for the 6th axis
#define JOINT6_ENABLE_PIN      53  // Update with the correct pin for the 6th axis

AccelStepper joint1(1, JOINT1_STEP_PIN, JOINT1_DIR_PIN);
AccelStepper joint2(1, JOINT2_STEP_PIN, JOINT2_DIR_PIN);
AccelStepper joint3(1, JOINT3_STEP_PIN, JOINT3_DIR_PIN);
AccelStepper joint4(1, JOINT4_STEP_PIN, JOINT4_DIR_PIN);
AccelStepper joint5(1, JOINT5_STEP_PIN, JOINT5_DIR_PIN);
AccelStepper joint6(1, JOINT6_STEP_PIN, JOINT6_DIR_PIN);  // Create a new instance for the 6th axis

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

// Instantiate publisher (for debugging purposes)
//ros::Publisher steps("joint_steps_feedback",&msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps) {
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6; 
  joint_step[6] = arm_steps.position7; // gripper position <0-180>
}

void gripper_cb(const std_msgs::UInt16& cmd_msg) {
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  digitalWrite(13, HIGH - digitalRead(13));  // Toggle LED  
}

// Instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps", arm_cb); // subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); // subscribes to gripper position
// To publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  // Put your setup code here, to run once:
  // Serial.begin(57600);
  pinMode(13, OUTPUT);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  // nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(750);
  joint3.setMaxSpeed(2000);
  joint4.setMaxSpeed(500);
  joint5.setMaxSpeed(1000);
  joint6.setMaxSpeed(1000);  // Adjust the max speed for the 6th axis

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  steppers.addStepper(joint6);  // Add the 6th axis to the MultiStepper

  // Configure gripper servo
  gripper.attach(11);
  
  digitalWrite(13, 1); // Toggle LED
}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[6];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0];
    positions[1] = joint_step[1];
    positions[2] = joint_step[2];
    positions[3] = joint_step[3];
    positions[4] = joint_step[4];
    positions[5] = joint_step[5]; // 6th axis

    // Publish back to ROS to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    gripper.write(joint_step[5]);  // Move gripper after manipulator reaches the goal   
  }
  digitalWrite(13, HIGH - digitalRead(13)); // Toggle LED
  joint_status = 0;
  
  nh.spinOnce();
  delay(1);
}
