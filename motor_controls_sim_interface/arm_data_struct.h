#ifndef _ARM_DATA_H_
#define _ARM_DATA_H_

typedef struct rec_arm_data_struct rec_arm_data;
typedef struct store_arm_data_struct store_arm_data;
typedef struct pregame_arm_data_struct pregame_arm_data;

// raspberry pi analyzes positions of both arms and sends their coordinates
// through usb serial to arduino when user presses calibrate button
// or just turns on
struct pregame_arm_data_struct {
  float arm0_theta0;
  float arm0_theta1;
  float arm1_theta0;
  float arm1_theta1;
};

// raspberry pi sends desired position and velocity of arm's end effector
// and arduino will drive motor movements to produce this
struct rec_arm_data_struct {
  float arm_x;
  float arm_y;
  float arm_vx;
  float arm_vy;
};

// meant to store info about a robot arm, updated by arduino exclusively
struct store_arm_data_struct {
  float arm_x;
  float arm_y;
  float arm_theta0;
  float arm_theta1;
  // NOTE: Need to worry about angle of each subarm probably

  int motor0_step_pin;
  int motor1_step_pin;

  int motor0_dir_pin;
  int motor1_dir_pin;

  float motor0_speed;
  float motor1_speed;
};

// hacky way to prevent accelStepper library from deccelerating arms as they
// approach goal. Purpose is for arm to reach goal with a target velocity and
// to have arm maintain this speed as it nears its target, need to provide
// accelStepper classes wtih a somewhat unreachable goal.
struct joint_goals_struct {
  float arm0_theta0;
  float arm0_theta1;
  float arm1_theta0;
  float arm1_theta1;
}

#endif
