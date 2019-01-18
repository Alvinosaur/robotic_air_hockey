#include <stdlib.h>
#include <math.h>
#include <ros.h>
#include <airhockey_main/ArmAngles.h>
#include <AccelStepper.h>

#include "arm_data_struct.h"

#define LENGTH (float)5  // inches
#define SQ(x) (float)(x*x)
#define STEPS_PER_ROT (float)3200  // depends on motor driver used
#define RAD_TO_STEP(x) x*(STEPS_PER_ROT/(2*PI))
#define MAX_ERROR 2
#define ROS_UPDATE_INTERVAL 300

// Pin Connections for the Easy Drivers
// Odd One:
// Step - 1
// Dir - 0
// Top Left: 
// Step - 10 
// Dir - 9
// Bottom Left:
// Step - 13
// Dir - 12
// Bottom Right:
// Step - 4
// Dir - 3


// ros setup
ros::NodeHandle nh;

// arm1(left) and arm2(right) setup
AccelStepper arm0_stepper0(AccelStepper::DRIVER, 2, 3);
AccelStepper arm0_stepper1(AccelStepper::DRIVER, 4, 5);
AccelStepper arm1_stepper0(AccelStepper::DRIVER, 6, 7);
AccelStepper arm1_stepper1(AccelStepper::DRIVER, 8, 9);

// globals
struct joint_goals_struct joint_goals;
bool calibrated = false;
long prev_ros_update;

// callback that receives ground-truth arm angles from raspberry pi
void receive_data_cb(airhockey_main::ArmAngles& arm_data)
{
  if (arm_data.is_goal) set_goals(arm_data);
  else if (arm_data.success_calib) calib_arms(arm_data);
  else {
    // some warning LED or display that says calibration incomplete
    // maybe force user to recalibrate by pressing some button again
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

// Assumes incoming angles are in RADIANS
void calib_arms(airhockey_main::ArmAngles& arm_data)
{
  arm0_stepper0.setCurrentPosition(RAD_TO_STEP(arm_data.arm0_joint0));
  arm0_stepper1.setCurrentPosition(RAD_TO_STEP(arm_data.arm0_joint1));
  arm1_stepper0.setCurrentPosition(RAD_TO_STEP(arm_data.arm1_joint0));
  arm1_stepper1.setCurrentPosition(RAD_TO_STEP(arm_data.arm1_joint1));
  calibrated = true;
}

void set_goals(airhockey_main::ArmAngles& arm_data)
{
  // don't want to set same goal for accelStepper classes since we don't
  // want decceleration when reaching goal
  joint_goals.arm0_theta0 = int(RAD_TO_STEP(arm_data.arm0_joint0));
  joint_goals.arm0_theta1 = int(RAD_TO_STEP(arm_data.arm0_joint1));
  joint_goals.arm1_theta0 = int(RAD_TO_STEP(arm_data.arm1_joint0));
  joint_goals.arm1_theta1 = int(RAD_TO_STEP(arm_data.arm1_joint1));

  // Acceleration really determines everything
  arm0_stepper0.setAcceleration(int(abs(RAD_TO_STEP(arm_data.arm0_accel0))));
  arm0_stepper0.setAcceleration(int(abs(RAD_TO_STEP(arm_data.arm0_accel1))));
  arm1_stepper1.setAcceleration(int(abs(RAD_TO_STEP(arm_data.arm1_accel0))));
  arm1_stepper1.setAcceleration(int(abs(RAD_TO_STEP(arm_data.arm1_accel1))));

  // max speed will be the goal speed to reach at point
  arm0_stepper0.setMaxSpeed(int(abs(RAD_TO_STEP(arm_data.arm0_omega0))));
  arm0_stepper0.setMaxSpeed(int(abs(RAD_TO_STEP(arm_data.arm0_omega1))));
  arm0_stepper1.setMaxSpeed(int(abs(RAD_TO_STEP(arm_data.arm1_omega0))));
  arm1_stepper1.setMaxSpeed(int(abs(RAD_TO_STEP(arm_data.arm1_omega1))));
}

void wait_for_calib()
{
  // show some LED or text on an LCD screen notifying players
  digitalWrite(LED_BUILTIN, HIGH);
  while(!calibrated) ;  // check if this loop slows down Arduino
  digitalWrite(LED_BUILTIN, LOW);
}

// set default max speed and acceleration for the four motors
void config_motors()
{
  arm0_stepper0.setMaxSpeed(5000);  // max 10,000 steps/s
  arm0_stepper0.setAcceleration(5000);
  arm0_stepper1.setMaxSpeed(5000);
  arm0_stepper1.setAcceleration(5000);
  arm1_stepper0.setMaxSpeed(5000);
  arm1_stepper0.setAcceleration(5000);
  arm1_stepper1.setMaxSpeed(5000);
  arm1_stepper1.setAcceleration(5000);
}

ros::Subscriber<airhockey_main::ArmAngles> \
  recalib_sub("arm_angles_topic", &receive_data_cb);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  prev_ros_update = millis();
  nh.initNode();
  nh.subscribe(recalib_sub);

  config_motors();
  wait_for_calib();
}

void check_move_arm0(AccelStepper arm, float goal)
{
  float theta_diff = goal - arm.currentPosition();
  if (abs(theta_diff) > MAX_ERROR) {
    if (theta_diff > 0) arm.moveTo(10000);  // rotate relative to current step
    else arm.moveTo(-10000);
    arm.run();
  }
}

void loop()
{
  // only update arm data after ROS_UPDATE_INTERVAL (ms) amount of time
  if (millis() - prev_ros_update > ROS_UPDATE_INTERVAL) {
    nh.spinOnce();
    prev_ros_update = millis();
  }
  check_move_arm(arm0_stepper0, joint_goals.arm0_theta0);
  check_move_arm(arm0_stepper1, joint_goals.arm0_theta1);
  check_move_arm(arm1_stepper0, joint_goals.arm1_theta0);
  check_move_arm(arm1_stepper1, joint_goals.arm1_theta1);

}
