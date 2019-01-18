from tkinter import *
import copy
import math
import random
import time
import sys

import utilities as util
import motion_planning as motion  # actual functions driving robot movement

import rospy
from airhockey_main.msg import ArmAngles
from airhockey_main_script import publish_arm_data


# Credit to the 15-112 Course at Carnegie Mellon University for providing
# example graphics functions with tkinter
# Link: https://www.cs.cmu.edu/~112/notes/notes-animations-part2.html
# this script demonstrates the correct determination of puck velocity vector
# intersection with bounds of robot arm

NUM_LINKS = 2
SEC_TO_MILLIS = float(1)/1000
DEG_TO_RAD = math.pi/180
SC_WIDTH = 200
SC_HEIGHT = 400
EXTENT_CHECK = float(3)/4  # note: reversed for graphics, -> 1 means closer to arms
root = Tk()


## Drawing ##
def init(data):
    # CONSTANTS
    # arm starts at bottom center of screen, perspective of robot
    # note in graphics, y must decrease for arm to point upwards
    data.arm0_x = SC_WIDTH/2
    data.arm0_y = SC_HEIGHT
    data.ball_r = 10
    data.dot_r = 1
    data.ball_orig_x = SC_WIDTH/2  # starting position of ball
    data.ball_orig_y = data.ball_r
    data.ball_vel = 5
    data.arm_length = 50
    data.line_width = 5
    data.timerDelay = 25  # 25ms delay between each updated frame
    data.arm_color = "red"
    data.ball_color = "yellow"
    data.dot_color = "green"
    data.real_arm_bounds_color = "orange"
    data.deflect_traj_color = "purple"
    data.bound_color = "black"

    # VARIABLES
    # so all angles in degrees so can increment/decrement easily
    data.arm0_theta0 = 0
    data.arm0_theta1 = 0
    data.omega0 = 1 * DEG_TO_RAD / data.timerDelay
    data.omega1 = 1 * DEG_TO_RAD / data.timerDelay
    data.acc0 = 1 * DEG_TO_RAD / data.timerDelay
    data.acc1 = 1 * DEG_TO_RAD / data.timerDelay
    data.goal0 = 45
    data.goal1 = 45
    data.ball_x = data.ball_orig_x  # actual location of ball, center
    data.ball_y = data.ball_orig_y
    data.ball_goal_x = 0
    data.ball_goal_y = 0
    approx_vel(data)  # modifies data.ball_vx, data.ball_vy
    data.collision_x, data.collision_y = 0, 0
    data.pred_x, data.pred_y = 0, 0
    data.pred_vx, data.pred_vy = 0, 0
    data.reach_bound = 0
    data.time_to_collision = 0
    data.reached_goal = False
    data.move_ball = False
    data.deflections = []


def timerFired(data):
    if data.move_ball:
        if ((data.ball_x + data.ball_r + data.ball_vx > SC_WIDTH) or
                data.ball_x - data.ball_r + data.ball_vx < 0):
            data.ball_vx *= -1  # bounce with opposite velocity
        else:
            data.ball_x += data.ball_vx
        if data.ball_y + data.ball_r > SC_HEIGHT:
            data.ball_x = data.ball_orig_x  # actual location of ball, center
            data.ball_y = data.ball_orig_y
            data.move_ball = False
        else:
            data.ball_y += data.ball_vy
    if not data.reached_goal:
        move_arm(data)


def keyPressed(event, data):
    # press Q or q to quit
    if (event.char == 'q') or (event.char == 'Q'):
        root.destroy()
        sys.exit()
    elif (event.char == 'l') or (event.char == 'L'):
        data.move_ball = not data.move_ball


def mousePressed(event, data):
    data.ball_goal_x = event.x
    data.ball_goal_y = event.y
    approx_vel(data)

    # un-transformed data
    puck_pose = util.Struct()
    puck_pose.x = data.ball_x
    puck_pose.y = data.ball_y
    puck_pose.vx = data.ball_vx
    puck_pose.vy = data.ball_vy

    # store table info
    table = util.Struct()
    table.width = SC_WIDTH
    table.length = SC_HEIGHT

    # store arm info
    arm = util.Struct()
    arm.x = data.arm0_x
    arm.y = util.transform(data.arm0_y, True, SC_HEIGHT)
    arm.num_links = NUM_LINKS
    arm.link_length = data.arm_length
    arm.theta0 = data.arm0_theta0 * DEG_TO_RAD
    arm.theta1 = data.arm0_theta1 * DEG_TO_RAD
    arm.speed = 5

    # Note: to show linearized trajectory, modify the below function to also
    # return lin_trajectory
    rand_prop = float(random.randint(90, 95))/ 100
    collision_info, deflections, joint_info = (
            motion.predict_puck_motion(table, arm, puck_pose, 0.95))

    data.deflections = transform_deflections(deflections)
    data.collision_x = collision_info.x
    data.collision_y = util.transform(collision_info.y, True, SC_HEIGHT)
    data.time_to_collision = collision_info.time_to_collision

    # maybe need some conversion ratio
    data.omega0 = joint_info.omega0
    data.omega1 = joint_info.omega1
    data.acc0 = joint_info.acc0
    data.acc1 = joint_info.acc1


    data.goal0 = joint_info.joint0 / DEG_TO_RAD
    data.goal1 = joint_info.joint1 / DEG_TO_RAD
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
        first_collision = data.deflections[0]
        canvas.create_line(data.ball_x, data.ball_y,
                           first_collision[0], first_collision[1],
                           fill=data.deflect_traj_color, width=data.line_width)

        # Draw last deflection trajectory from last
        last_deflection = data.deflections[-1]
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
def approx_vel(data):
    dist = ((data.ball_goal_x - data.ball_orig_x)**2 +
            (data.ball_goal_y - data.ball_orig_y)**2)**0.5
    # mult by -1 to account for graphics coordinate frame
    ang = math.atan2((data.ball_goal_y - data.ball_orig_y),
                     data.ball_goal_x - data.ball_orig_x)

    data.ball_vx = data.ball_vel * math.cos(ang)
    # -1 to account for graphics inversion of coordinate frame
    data.ball_vy = data.ball_vel * math.sin(ang)


def transform_deflections(deflections):
    for deflect in deflections:
        deflect[1] = util.transform(deflect[1], True, SC_HEIGHT)
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
    if not reached_goal0:
        data.omega0 += data.acc0
        data.arm0_theta0 += data.omega0 / DEG_TO_RAD

    # joint1
    if not reached_goal1:
        data.omega1 += data.acc1
        data.arm0_theta1 += data.omega1 / DEG_TO_RAD

    # finally reached goal when both joints have reached their goal angles
    data.reached_goal = reached_goal0 and reached_goal1


run(SC_WIDTH, SC_HEIGHT*2)
