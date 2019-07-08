from tkinter import *
import copy
import math
import random
import time
import sys

import utilities as util
import motion_planning as motion  # actual functions driving robot movement

# Credit to the 15-112 Course at Carnegie Mellon University for providing
# example graphics functions with tkinter
# Link: https://www.cs.cmu.edu/~112/notes/notes-animations-part2.html
# this script demonstrates the correct determination of puck velocity vector
# intersection with bounds of robot arm
LINK_LENGTH = 6 / float(36) # 6 inches to meters
MAX_ARM_SPEED = 0.5  # m/s
BALL_VEL = .75  # m/s
TIME_DELAY = 2
TIME_UPDATE_SCALE = float(40)
M_TO_PIX = float(300)  # meters to pixels
NUM_LINKS = 2
HOME_0 = 45
HOME_1 = 150
SEC_TO_MILLIS = float(1)/1000
DEG_TO_RAD = math.pi / 180
RAD_TO_DEG = 1 / DEG_TO_RAD
SC_WIDTH = 200
SC_HEIGHT = 400

# store table info in meters
table = util.Struct()
table.width = SC_WIDTH / M_TO_PIX
table.length = SC_HEIGHT / M_TO_PIX
EXTENT_CHECK = float(3)/4  # note: reversed for graphics, -> 1 means closer to arms
DEBUG = True
root = Tk()

"""All angles running the simulator are in degrees for easy decrement/increment.

Coordinate Frame of Simulator:
                    __________X
                    |
                    |
                    |
                  Y |

Coordinate Frame of Calculations:
                  Y |
                    |
                    |
                    |_________X

Returns:
    [type] -- [description]
"""

def initBall(data):
    # ball starts at top-center of screen by default
    data.ball_orig_x = SC_WIDTH/2
    data.ball_orig_y = 10
    data.ball_vel = BALL_VEL * M_TO_PIX
    data.ball_r = 10  
    data.ball_x = data.ball_orig_x
    data.ball_y = data.ball_orig_y
    data.ball_goal_x, data.ball_goal_y = data.ball_x, data.ball_y
    data.ball_vx, data.ball_vy = 0, 0
    data.ball_color = "yellow"


def initArm(data):
    # arm starts at bottom center of screen, perspective of robot
    data.arm0_x = SC_WIDTH/2
    data.arm0_y = SC_HEIGHT
    data.arm_length = LINK_LENGTH * M_TO_PIX
    data.arm0_theta0, data.arm0_theta1 = 45, 135  # base and 2nd link angle
    data.omega0, data.omega1 = 0, 0  # base and 2nd link angular vel
    data.acc0, data.acc1 = 0, 0  # base and 2nd link angular acc
    data.goal0, data.goal1 = 45, 45  # initial
    data.collision_x, data.collision_y = 0, 0
    data.pred_x, data.pred_y = 0, 0
    data.pred_vx, data.pred_vy = 0, 0
    data.arm_color = "red"


def init(data):
    initBall(data)
    initArm(data)
    # Drawing Constants
    data.dot_r = 1
    data.line_width = 5
    data.timerDelay = 25  # 25ms delay between each updated frame
    data.dot_color = "green"
    data.real_arm_bounds_color = "orange"
    data.deflect_traj_color = "purple"
    data.bound_color = "black"

    # Dynamic Variables
    data.time_to_collision = 0
    data.reached_goal = True
    data.return_home = False
    data.move_ball = False
    data.deflections = []


def timerFired(data):
    if data.move_ball:
        if ((data.ball_x + data.ball_r > SC_WIDTH) or
                data.ball_x - data.ball_r < 0):
            data.ball_vx *= -1  # bounce with opposite velocity
            if len(data.deflections) > 0: data.deflections.pop()
        if data.ball_y + data.ball_r > SC_HEIGHT:
            data.ball_x = data.ball_orig_x  # actual location of ball, center
            data.ball_y = data.ball_orig_y
            data.move_ball = False
        data.ball_y += data.ball_vy / TIME_UPDATE_SCALE
        data.ball_x += data.ball_vx / TIME_UPDATE_SCALE

    if not data.reached_goal:
        move_arm(data)


def keyPressed(event, data):
    # press Q or q to quit
    if (event.char == 'q') or (event.char == 'Q'):
        root.destroy()
        sys.exit()
    elif (event.char == 'p') or (event.char == 'P'):
        data.move_ball = not data.move_ball


def get_puck_world_frame(data):
    # store puck pose in global frame
    puck_pose = util.Struct()
    puck_pose.x = data.ball_x / M_TO_PIX
    puck_pose.y = util.transform(data.ball_y, True, SC_HEIGHT) / M_TO_PIX
    puck_pose.vx = data.ball_vx / M_TO_PIX
    puck_pose.vy = util.transform(data.ball_vy, False) / M_TO_PIX
    if DEBUG: 
        print('[DEBUG] Puck x, y, vx, vy: %.2f, %.2f, %.2f, %.2f' % (
            puck_pose.x, puck_pose.y, puck_pose.vx, puck_pose.vy))
    return puck_pose


def get_arm_world_frame(data):
    # store arm info in global frame
    arm = util.Struct()
    arm.x = data.arm0_x / M_TO_PIX
    arm.y = util.transform(data.arm0_y, True, SC_HEIGHT) / M_TO_PIX
    arm.num_links = NUM_LINKS
    arm.link_length = data.arm_length / M_TO_PIX
    arm.theta0 = data.arm0_theta0 * DEG_TO_RAD
    arm.theta1 = data.arm0_theta1 * DEG_TO_RAD
    arm.speed = MAX_ARM_SPEED
    if DEBUG: 
        print('[DEBUG] Arm x, y, Angle 0, Angle 1: %.2f, %.2f, %.2f, %.2f' % (
            arm.x, arm.y, arm.theta0, arm.theta1))
    return arm


def mousePressed(event, data):
    # If ball not already moving
    if not data.move_ball:
        data.move_ball = True
        approx_vel(data, event.x, event.y)

    puck_pose = get_puck_world_frame(data)
    arm = get_arm_world_frame(data)

    # Note: to show linearized trajectory, modify the below function to also
    # return lin_trajectory
    collision_info, deflections, joint_info = (
            motion.predict_puck_motion(table, arm, puck_pose, 0.95))

    # reverse so can pop off deflections in chronological order O(1)
    data.deflections = transform_deflections(deflections)
    data.deflections.reverse()
    data.collision_x = collision_info.x * M_TO_PIX
    data.collision_y = util.transform(collision_info.y * M_TO_PIX, 
                                    True, SC_HEIGHT)
    data.time_to_collision = collision_info.time_to_collision * TIME_DELAY
    if DEBUG:
        print('[DEBUG] Collision x, y, t: %.2f, %.2f, %.2f' % (
            data.collision_x, data.collision_y, data.time_to_collision))

    data.omega0 = joint_info.omega0 * RAD_TO_DEG
    data.omega1 = joint_info.omega1 * RAD_TO_DEG
    data.acc0 = joint_info.acc0 * RAD_TO_DEG
    data.acc1 = joint_info.acc1 * RAD_TO_DEG

    data.goal0 = (joint_info.joint0 * RAD_TO_DEG) % 360
    data.goal1 = (joint_info.joint1 * RAD_TO_DEG) % 360

    data.reached_goal = False

def draw_bounds(canvas, data):
    canvas.create_line(SC_WIDTH, 0, SC_WIDTH, SC_HEIGHT,
                       fill=data.bound_color, width=data.line_width)


def draw_arm(canvas, data):
    end_x0 = data.arm0_x + data.arm_length * math.cos(data.arm0_theta0 * DEG_TO_RAD)
    end_y0 = data.arm0_y - data.arm_length * math.sin(data.arm0_theta0 * DEG_TO_RAD)

    # start of second link is the end of first link
    # note that the overall angle for the second joint is the combined angle of
    # the first joint and second joint
    end_x1 = end_x0 + data.arm_length * (
                math.cos((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))
    end_y1 = end_y0 - data.arm_length * (
                math.sin((data.arm0_theta0 + data.arm0_theta1) * DEG_TO_RAD))

    #draw first link
    canvas.create_line(data.arm0_x, data.arm0_y,  # starting x, y
                       end_x0, end_y0,            # ending x, y
                       fill=data.arm_color,           # color
                       width=data.line_width)      # width of line

    # draw second link
    canvas.create_line(end_x0, end_y0,
                       end_x1, end_y1,
                       fill=data.arm_color, width=data.line_width)


def draw_ball_and_collision(canvas, data):
    canvas.create_oval(data.ball_x - data.ball_r, data.ball_y - data.ball_r,
                       data.ball_x + data.ball_r, data.ball_y + data.ball_r,
                       fill=data.ball_color)
    canvas.create_oval(data.collision_x - data.dot_r,
                       data.collision_y - data.dot_r,
                       data.collision_x + data.dot_r,
                       data.collision_y + data.dot_r,
                       fill=data.dot_color)


def draw_perimeter(canvas, data):
    canvas.create_oval(data.arm0_x - 2 * data.arm_length,
                       data.arm0_y - 2 * data.arm_length,
                       data.arm0_x + 2 * data.arm_length,
                       data.arm0_y + 2 * data.arm_length,
                       outline=data.real_arm_bounds_color)


def draw_trajectory(canvas, data):
    # Draw simplified trajectory of puck without bounces
    # Vector determined in motion.linearize_trajectory()
    # canvas.create_line(data.pred_x, SC_HEIGHT - data.pred_y,
    #                    data.collision_x, data.collision_y,
    #                    fill=data.pred_traj_color, width=data.line_width)

    # If no collisions, draw from puck to collision directly
    if len(data.deflections) == 0:
        canvas.create_line(data.ball_x, data.ball_y,
                           data.collision_x, data.collision_y,
                           fill=data.deflect_traj_color, width=data.line_width)
    else:
        # Draw first deflection trajectory from puck position to collision
        first_collision = data.deflections[-1]
        canvas.create_line(data.ball_x, data.ball_y,
                           first_collision[0], first_collision[1],
                           fill=data.deflect_traj_color, width=data.line_width)

        # Draw last deflection trajectory from last
        last_deflection = data.deflections[0]
        canvas.create_line(last_deflection[0], last_deflection[1],
                           data.collision_x, data.collision_y,
                           fill=data.deflect_traj_color, width=data.line_width)

        # Draw all other collision trajectories between two collision points
        for inc in range(len(data.deflections) - 1):
            prev_collision = data.deflections[inc]
            next_collision = data.deflections[inc+1]
            canvas.create_line(prev_collision[0], prev_collision[1],
                               next_collision[0], next_collision[1],
                               fill=data.deflect_traj_color, width=data.line_width)


def redrawAll(canvas, data):
    draw_arm(canvas, data)
    draw_bounds(canvas, data)
    draw_perimeter(canvas, data)
    draw_trajectory(canvas, data)
    draw_ball_and_collision(canvas, data)
    canvas.create_text(data.width/2, 50,
                       text="Each Arm Length: %d" % data.arm_length)
    canvas.create_text(data.width/2, data.height/2,
                       text="theta0: %f" % data.arm0_theta0)
    canvas.create_text(data.width/2, data.height/2 + 50,
                       text="theta1: %f" % data.arm0_theta1)


def run(width=300, height=300):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='white', width=0)
        redrawAll(canvas, data)
        canvas.update()

    def timerFiredWrapper(canvas, data):
        timerFired(data)
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    # Set up data and call init
    data = util.Struct()
    data.width = width
    data.height = height
    root.resizable(width=False, height=False) # prevents resizing window
    init(data)
    # create the root and the canvas
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    root.bind("<Key>", lambda event:
        keyPressedWrapper(event, canvas, data))
    root.bind("<Button-1>", lambda event:
        mousePressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    print('Press q to quit')
    root.mainloop()  # blocks until window is closed
    print("bye!")


## Logic and Calculations ##
def approx_vel(data, x_g, y_g):
    ang = math.atan2((y_g - data.ball_orig_y),
                     x_g - data.ball_orig_x)

    print(ang)
    data.ball_vx = data.ball_vel * math.cos(ang)
    # -1 to account for graphics inversion of coordinate frame
    data.ball_vy = data.ball_vel * math.sin(ang)


def transform_deflections(deflections):
    """Transform global frame coordinates in meters to simulation frame in 
    pixels.
    
    Arguments:
        deflections {List} -- list of x, y coordinates of puck collisions with 
        wall.
    
    Returns:
        List -- transformed deflections list, destructively modified.
    """
    for deflect in deflections:
        deflect[0] *= M_TO_PIX
        deflect[1] = util.transform(deflect[1] * M_TO_PIX, True, SC_HEIGHT)
    return deflections


def find_orig_deflect_intersect(data):
    try:
        time_collide = (data.pred_x - data.ball_x) / (data.ball_vx - data.pred_vx)
    except ZeroDivisionError:
        return 0, 0, False
    collide_x = data.ball_x + data.ball_vx * time_collide
    collide_y = data.ball_y + data.ball_vy * time_collide
    return collide_x, collide_y, True


def check_reached_goal(cur, goal):
    # error less than 1 deg is good enough
    return abs(cur - goal) < 5


def move_arm(data):
    # Note: there is an illusion sometimes that only one arm moves at a time
    # but when both joint angles have opposite change (one dec, the other inc)
    # and the second link will look like it's staying in place
    # joint0
    reached_goal0 = check_reached_goal(data.arm0_theta0, data.goal0)
    reached_goal1 = check_reached_goal(data.arm0_theta1, data.goal1)
    if DEBUG:
        print('[DEBUG] Joint 0 real, goal, omeg, acc: %.2f, %.2f, %.2f, %.2f' %     (data.arm0_theta0, data.goal0, data.omega0, data.acc0))
        print('[DEBUG] Joint 1 real, goal, omeg, acc: %.2f, %.2f, %.2f, %.2f' %     (data.arm0_theta1, data.goal1, data.omega1, data.acc1))
    if not reached_goal0:
        data.omega0 += data.acc0 / TIME_UPDATE_SCALE
        data.arm0_theta0 += data.omega0 / TIME_UPDATE_SCALE

    # joint1
    if not reached_goal1:
        data.omega1 += data.acc1 / TIME_UPDATE_SCALE
        data.arm0_theta1 += data.omega1 / TIME_UPDATE_SCALE

    # finally reached goal when both joints have reached their goal angles
    if reached_goal0 and reached_goal1:
        if data.return_home:  # if reached goal home position, done
            data.reached_goal = True
            data.return_home = False
        else:
            data.return_home = True
            data.goal0 = HOME_0
            data.goal1 = HOME_1


run(SC_WIDTH, SC_HEIGHT*2)
