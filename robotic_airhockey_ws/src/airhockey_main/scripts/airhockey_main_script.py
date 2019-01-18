#!/usr/bin/env python
import copy
import cv2 as cv
import numpy as np
import rospy
from airhockey_main.msg import ArmAngles
import math
import time

import motion_planning as motion
import utilities as util
from TrackPuck import Puck
# import calibArm

# conversion ratio from inches to pixels
# all other lengths defined in pixels
INCH_TO_PIX = float(1200)/38  # we used width of table for this
VEL_Y_THRESHOLD_RATIO = .0025
TABLE_LENGTH = 800 # pixels
TABLE_WIDTH = 400
NUM_LINKS = float(2)
LINK_LENGTH = float(9) * INCH_TO_PIX  # inches
CAM_INDEX = 1  # Change to 1 for ps3
DEFAULT_ACCEL = 5000  # steps/s^2
DEFAULT_VEL = 7000
STEPS_TO_RAD = 2*math.pi/1600
RAD_TO_DEG = 180/math.pi

table = util.Struct()
table.length = TABLE_LENGTH
table.width = TABLE_WIDTH

arm0 = util.Struct()
arm0.x = 270
arm0.y = 0
arm0.num_links = 2
arm0.link_length = LINK_LENGTH
arm0.joint0 = math.pi/2  # will be immediately overwritten after calling calib script
arm0.joint1 = 0

arm1 = util.Struct()
arm1.x = 120
arm1.y = 0
arm1.num_links = 2
arm1.link_length = LINK_LENGTH
arm1.joint0 = math.pi/2  # will be immediately overwritten after calling calib script
arm1.joint1 = 0

def publish_arm_data(publisher, is_goal=True, success=False,
                     angles=None, vel=None, acc=None):

    """
    Takes in all necessary arm_data and publishes it to arduino. Serves as main
    communication node with arduino regardless of purpose.

    :param publisher: ROS publisher to send arm data to arduino
    :type: rospy.Publisher

    :param arm_data: arm data
    :type: dict

    """
    arm_data = ArmAngles()
    arm_data.success = success
    arm_data.is_goal = is_goal

    if angles != None:
        arm_data.arm0_joint0 = angles.arm0_joint0
        arm_data.arm0_joint1 = angles.arm0_joint1
        arm_data.arm1_joint0 = angles.arm1_joint0
        arm_data.arm1_joint1 = angles.arm1_joint1

    if vel != None:
        arm_data.arm0_omega0 = vel.arm0_omega0
        arm_data.arm0_omega1 = vel.arm0_omega1
        arm_data.arm1_omega0 = vel.arm1_omega0
        arm_data.arm1_omega1 = vel.arm1_omega1

    if acc != None:
        arm_data.arm0_accel0 = acc.arm0_accel0
        arm_data.arm0_accel1 = acc.arm0_accel1
        arm_data.arm1_accel0 = acc.arm1_accel0
        arm_data.arm1_accel1 = acc.arm1_accel1

    # add acceleration and velocity if we solve these
    publisher.publish(arm_data)

def anticipate_puck_mode(puck_pose, publisher):
    # Note: to show linearized trajectory, modify the below function to also
    # return lin_trajectory
    collision_info, deflections, joint_info = (
        motion.predict_puck_motion(table, arm0, puck_pose))
    # draw_deflections(deflections)
    accel0, accel1 = (
            simple_move(arm0, joint_info, collision_info.time_to_collision))

    angles = util.Struct()
    angles.arm0_joint0 = joint_info.joint0
    angles.arm0_joint1 = joint_info.joint1
    angles.arm1_joint0 = -1*math.pi/4
    angles.arm1_joint1 = 0

    vel = None

    acc = util.Struct()
    acc.arm0_accel0 = accel0
    acc.arm0_accel1 = accel1
    acc.arm1_accel0 = 0
    acc.arm1_accel1 = 0

    publish_arm_data(publisher, False, True, angles, vel, acc)

    #                  arm0_joint0=arm0.joint0, arm0_joint1=arm0.joint1,
    #                  arm1_joint0=arm1.joint0, arm1_joint1=arm1.joint1

def simple_move(current_joint_pose, joint_goals, collision_time):
    # NOTE: account for division by zero error
    accel0 = ((2 * (current_joint_pose.joint0 - joint_goals.joint0)) /
                collision_time**2)
    accel1 = ((2 * (current_joint_pose.joint1 - joint_goals.joint1)) /
                collision_time**2)
    return accel0, accel1
    # after calling this function, arm should have moved to new location, so
    # update best estimate of arm. However, once in a while(maybe after 5 calls
    # to a move function), calibrate arm by publishing arm angle information
    # update_joint_info()


def extreme_defense_mode(publisher, puck_pose):
    """
    Don't even move in the y-direction, only have arms move horizontally to
    block puck
    """
    print('x, y, vx, vy: ', puck_pose.x, puck_pose.y, puck_pose.vx,
            puck_pose.vy)
    angles = util.Struct()
    try:
        theta0, theta1 = motion.calc_joints_from_pos(arm0.link_length, puck_pose.x,
                                                50)
        angles.arm0_joint0 = theta0
        arm0.joint0 = theta0
        arm0.joint1 = theta1
        angles.arm0_joint1 = theta1
        angles.arm1_joint0 = arm1.joint0
        angles.arm1_joint1 = arm1.joint1
        print('move arm0 to: ', theta0 * RAD_TO_DEG, theta1 * RAD_TO_DEG)
    except:
        angles.arm0_joint0 = math.pi/2
        arm0.joint0 = math.pi/2
        arm0.joint1 = 0
        angles.arm0_joint1 = 0
        angles.arm1_joint0 = arm1.joint0
        angles.arm1_joint1 = arm1.joint1
    acc = util.Struct()
    acc.arm0_accel0 = DEFAULT_ACCEL * STEPS_TO_RAD
    acc.arm0_accel1 = DEFAULT_ACCEL * STEPS_TO_RAD
    acc.arm1_accel0 = DEFAULT_ACCEL * STEPS_TO_RAD
    acc.arm1_accel1 = DEFAULT_ACCEL * STEPS_TO_RAD


    vel = util.Struct()
    vel.arm0_omega0 = DEFAULT_VEL * STEPS_TO_RAD
    vel.arm0_omega1 = DEFAULT_VEL * STEPS_TO_RAD
    vel.arm1_omega0 = DEFAULT_VEL * STEPS_TO_RAD
    vel.arm1_omega1 = DEFAULT_VEL * STEPS_TO_RAD

    publish_arm_data(publisher, True, False,
                     angles, vel, acc)
    # time.sleep(2)


def defense_mode(publisher, arm, puck_pose):
    """
    Defense mode picks the best arm to use for a given situation avoid a
    collision between both arms. Then simply commands that arm to move to the
    anticipated puck location without any regard for puck's recoil velocity.

    This mode is triggered if the velocity of incoming puck is very high and
    would be too fast for any special path.

    :param publisher: ROS publisher to send arm data to arduino
    :type: rospy.Publisher

    :param arm_data: holds arm data, use of info, reliability bool
    :type: util.Struct()

    :param puck_pose: holds current puck position and velocity x,y
    :type: util.Struct()

    """
    # is_goal is checked first by Arduino, so success is unused for seting goals
    arm_msg.is_goal = True
    collision_info, deflections, joint_info = motion.predict_puck_motion(table, arm, puck_pose)
    if (is_first_arm):
        arm_msg.arm0_joint0 = theta0
        arm_msg.arm0_joint11 = theta1
    else:
        arm_data['arm1_joint0'] = theta0
        arm_data['arm1_joint1'] = theta1
    publish_arm_data(publisher, arm_data)

def attack_mode():
    """
    Use attack_mode if puck is moving towards robot at slow speed, allowing
    robot to get time to set up a path.
    """
    return

def hybrid_mode():
    """
    Maybe a hybrid between attack and defense mode. Have the arm first slow down
    the puck, then recoil and hit the puck.

    """
    return

def collab_mode():
    """
    """
    return


def troll_mode():
    """
    Using the predictive puck trajectory calculations, determine very early on
    whether the incoming puck will even enter the goal. If puck won't even enter
    the goal, then just have arms do nothing or position so that they don't
    touch the puck.

    """
    return

def fake_calib(publisher):
    angles = util.Struct()
    angles.arm0_joint0 = arm0.joint0
    # will be immediately overwritten after calling calib script
    angles.arm0_joint1 = arm0.joint1
    angles.arm1_joint0 = arm1.joint0
    angles.arm1_joint1 = arm1.joint1

    for i in range(10):
        publish_arm_data(publisher, False, True, angles)


def track_table(PuckTracker, cap):
    _, frame = cap.read()
    #frame = cv.imread("./table3.jpg")
    cv.circle(frame, (203, 104), 5, (0, 0, 255), -1)
    cv.circle(frame, (479, 90), 5, (0,  0, 255), -1)
    cv.circle(frame, (527, 466), 5, (0, 0, 255), -1)
    cv.circle(frame, (180, 480), 5, (0, 0, 255), -1)

    pts1 = np.float32([[203, 104], [479, 90], [527, 466], [180, 480]])
    pts2 = np.float32([[0, 380], [0, 0], [460, 0], [460, 380]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)

    result = cv.warpPerspective(frame, matrix, (460, 360))

    # cv.imshow("Frame", frame)
    # cv.imshow("Perspective transformation", result)

    PuckTracker.frame = result
    PuckTracker.track()
    key = cv.waitKey(1)
    if key == 27:
        return

    return PuckTracker


def legal_puck(puck_pose, prev_pose):
    if (puck_pose.x > 300 and puck_pose.y > 300): return False
    if (abs(puck_pose.x - prev_pose.x) > 100 or
        abs(puck_pose.y - prev_pose.y) > 100): return False
    return True


# MAIN FUNCTION THAT GETS CALLED ON RPI
def run_main():
    arm_data_pub = rospy.Publisher('arm_angles_topic', ArmAngles,
                                    queue_size=10)
    rospy.init_node('arm_data_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cap = cv.VideoCapture(CAM_INDEX)
    cap.open(CAM_INDEX)
    PuckTracker = Puck()
    puck_pose = util.Struct()
    fake_calib(arm_data_pub)
    temp = util.Struct()
    # NOTE: calibrate_arm is a blocking function that will only terminate and
    # return arm angles when arms are clear on table
    # arm0.joint0, arm0.joint1, arm1.joint0, arm1.joint1 = calibArm.calibrate_arm()

    # MAIN LOOP
    while not rospy.is_shutdown():
        prev_pose = copy.deepcopy(puck_pose)
        PuckTracker = track_table(PuckTracker, cap)
        temp.x = PuckTracker.y
        temp.y = PuckTracker.x
        temp.vx = PuckTracker.dy
        temp.vy = PuckTracker.dx
        if legal_puck(temp, prev_pose):
            puck_pose.x = temp.x
            puck_pose.y = temp.y
            puck_pose.vx = temp.vx
            puck_pose.vy = temp.vy
            extreme_defense_mode(arm_data_pub, puck_pose)
        # if util.is_puck_approaching(puck_pose.vy):
            # rate_class = (
            #         util.rate_puck_speed(vel_y, table_length, table_proportion))
            #     anticipate_puck_mode(puck_pose, arm_data_pub)
            # elif util.puck_slow():
            #     pass
            # else:  # puck is fast
            #     defense_mode(puck_pose, arm_data_pub)
        # print(puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy)
        # if (puck_pose.x != None and puck_pose.y != None):
        #     anticipate_puck_mode(puck_pose, arm_data_pub)
        # print(int(puck_pose.x), int(puck_pose.y), int(puck_pose.vx),
        #         int(puck_pose.vy))
        rate.sleep()


if __name__ == '__main__':
    try:
        run_main()
    except rospy.ROSInterruptException:
        pass

