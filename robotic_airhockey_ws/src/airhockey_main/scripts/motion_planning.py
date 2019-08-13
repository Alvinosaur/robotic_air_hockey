import copy
import math
import numpy as np

import utilities as util

MIN_V = .01  # m/s, or 1cm/s


def predict_puck_motion(table, arm, puck_pose, radius_prop=0.95):
    """
    This is the main function that will be called by the raspberry pi
    constantly. The raspberry pi will call computer vision scripts to return the
    puck's current location and velocity. This script calls many other functions
    and overall returns information such as the final collision of the puck
    with the extent of reach of the arm, all the deflections of the puck with
    the wall, and the desired joint angles for the robot arm.

    :param table:
    :type: Struct that contains the attributes:
        width: width of table
        length: length of table

    :param arm:
    :type: Struct that contains:
        x: base x of arm
        y: base y of arm
        theta0: base angle
        theta1: 2nd angle defined wrt base arm link
        link_length: length of one link(two links for our 2DOF robot arm)
        num_links: number of links for one arm (in our case 2)

    :param puck_pose:
    :type: Struct:
        x: center x of puck
        y: center y of puck
        vx: velocity x-component of puck
        vy: velocity y-component of puck

    :returns:
    """
    # note: graphics frame v.s real-world frame flipped on y-axis
    deflections = find_deflections(table, arm, puck_pose,
            radius_prop, [])

    lin_trajectory = linearize_trajectory(puck_pose, deflections)

    collision_info = vector_circle_intersect(arm, lin_trajectory, radius_prop)

    goal_joints = util.Struct()
    joint0, joint1 = calc_joints_from_pos(arm.link_length,
                        collision_info.x - arm.x,
                        (collision_info.y - arm.y))
    goal_joints.joint0 = joint0
    goal_joints.joint1 = joint1

    # NOTE: Passing in global collision location, not relative to base of arm
    omega0, omega1, acc0, acc1 = (
            calc_goal_joint_pose(arm, table, goal_joints, collision_info))

    goal_joints.omega0 = omega0
    goal_joints.omega1 = omega1
    goal_joints.acc0 = acc0
    goal_joints.acc1 = acc1

    return collision_info, deflections, goal_joints


def calc_joints_from_pos(arm_L, goal_x, goal_y):
    """
    Geometric solution to 2-DOF robot arm inverse kinematics. 
    Requires goal_y >= 0 and will always provide theta0 in bound [0, pi]
    to prevent arm from colliding with behind table edge.
    NOTE: goal_x and goal_y must be definied WITH RESPECT TO BASE OF ARM, so
    provide something like (arm_L, goal_x - base_x, goal_y - base_y)

    :param arm_L: length of robot arm (in our case, both links same length)
    :type: float

    :param goal_x: target x-position of end_effector
    :type: float

    :param goal_y: target y-position
    :type: float

    :returns (theta0, theta1): two joint angles required for goal position
    :type: tuple(float, float)

    """
    # while desired x, y is out of reach of arm
    # check if hypotenuse of triangle formed by x and y is > combined arm length
    try:
        theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_L**2) /
                       (2*arm_L**2))
    except:
        # floating point error where distance of goal from base of arm is only
        # slightly greater than reach of arm (ie: 100.000000000003)
        # solution: make arm slighty larger, almost same accuracy so good enough
        arm_L *= 1.01
        theta1 = math.acos((goal_x**2 + goal_y**2 - 2*arm_L**2) /
                       (2*arm_L**2))
    theta0 = math.atan2(goal_y, goal_x) - (
                math.atan2(arm_L*math.sin(theta1),
                           arm_L + arm_L*math.cos(theta1)))
    if not (0 <= theta0 and theta0 <= math.pi):
        theta1 *= -1
        theta0 = math.atan2(goal_y, goal_x) - (
                math.atan2(arm_L*math.sin(theta1),
                           arm_L + arm_L*math.cos(theta1)))
    assert(0 <= theta0 and theta0 <= math.pi)
    return (theta0, theta1)


def vector_circle_intersect(arm, p, radius_prop):
    """
    Finds intersection location (x, y) as well as time between incoming
    puck and perimeter of arm's max reach. Derviation shown on project page.
    Use a try-except block for both arms, pick the first one that has a valid
    solution.

    :param arm:
    :type: Struct that contains:
        x: base x of arm
        y: base y of arm
        theta0: base angle
        theta1: 2nd angle
        link_length: length of one link(two links for our 2DOF robot arm)
        num_links: number of links for one arm (in our case 2)

    :param p:
    :type: Struct:
        x: center x of puck
        y: center y of puck
        vx: velocity x-component of puck
        vy: velocity y-component of puck

    :param radius_prop: serves to prevent the second angle of the robot
    arm from ever reaching 0 (hence straight arm) since this can cause
    mechanical strain as well as cause singularities for the inversion of the
    jacobian. Must be between 0 and 1.
    :type: float

    :returns collision_info
    :type: Struct:
        x: x location of collision of arm with puck
        y: y location of collision of arm with puck
        t: time until collision

    :returns time_to_collision (milliseconds)
    :type: float

    """
    error_str = "Couldn't calculate new goal position for arm"
    # Quadratic formula
    arm_L = arm.link_length * arm.num_links * radius_prop
    a = (p.vx)**2 + (p.vy)**2
    b = ((2 * p.x * p.vx) - (2 * p.vx * arm.x) +
         (2 * p.y * p.vy) - (2 * p.vy * arm.y))
    c = (p.x - arm.x)**2 + (p.y - arm.y)**2 - arm_L**2
    sq_root_vals = b**2 - 4 * a * c

    if (sq_root_vals < 0) or (a == 0):
        raise util.NoSolutionError(error_str)
    else:
        time_to_collision = (-b - (sq_root_vals)**0.5) / (2 * a)
        if (time_to_collision < 0):
            time_to_collision = (-b + (sq_root_vals)**0.5) / (2 * a)
        goal_x = p.x + p.vx * time_to_collision
        goal_y = p.y + p.vy * time_to_collision

    if not (goal_y >= 0 and time_to_collision >= 0):
        raise util.NoSolutionError(error_str)

    # double-check that goal is within reach of arm
    # assert(((goal_x - base_x)**2 + (goal_y - base_y)**2)**0.5 < 2*arm_L)
    collision_info = util.Struct()
    collision_info.x = goal_x
    collision_info.y = goal_y
    collision_info.time_to_collision = time_to_collision

    return collision_info


def calc_goal_joint_pose(arm, table, goal_joints, goal_loc):
    """
    Calculates the desired velocity of arm when it reaches the goal position.
    Also calculates angular accelerations for both joints for them to reach
    end position at desired angle.

    :param arm: contain info on arm length, theta's, base x and y
    :type: float

    :param table_L: length of table
    :type: float

    :param goal_x, goal_y: goal x, y position of end effector
    :type: float

    """
    # Overall angle of velocity, not joint angles
    assert(table.length > goal_loc.y)
    theta_g = math.atan2(table.length - goal_loc.y, table.width/2 - goal_loc.x)
    arm_vel_g = np.array([
        [arm.speed * math.cos(theta_g)],
        [arm.speed * math.sin(theta_g)]
    ])

    # determine angular velocity of joints
     # determine angular velocity of joints
    jacobian = np.array([
        [-arm.link_length * math.sin(arm.theta0) -
            arm.link_length * math.sin(arm.theta0 + arm.theta1),
         -arm.link_length * math.sin(arm.theta0 + arm.theta1)],
        [arm.link_length * math.cos(arm.theta0) +
            arm.link_length * math.cos(arm.theta0 +arm.theta1),
         arm.link_length * math.cos(arm.theta0 + arm.theta1)]
    ])

    # maybe compute this from scratch
    try:
        inv_jacobian = np.linalg.inv(jacobian)
        omega = np.matmul(inv_jacobian, arm_vel_g)
    except:
        J_T = np.transpose(jacobian)
        cov = np.linalg.inv(np.matmul(J_T, jacobian))
        omega = np.matmul(np.matmul(cov, J_T), arm_vel_g)

    del_theta0 = goal_joints.joint0 - arm.theta0
    del_theta1 = goal_joints.joint1 - arm.theta1
    alpha0 = omega[0]**2 / (2 * del_theta0)
    alpha1 = omega[1]**2 / (2 * del_theta1)
    return omega[0], omega[1], alpha0, alpha1


def find_deflections(table, arm, puck_pose, radius_prop, deflections=[]):
    """
    Calculates the full trajectory of a puck including all its different
    deflections from wall sides. NOTE: modifies the puck_pose, so need to
    pass in a deep copy at the start. NOTE: requires the cartesian coordinate
    frame, so need to transform graphics coordinates to cartesian. Last 
    'deflection' is intersection with radius of arm.
    Cases:
        - puck bounces off wall like normal
        - puck never bounces off wall before reaching arm
        - puck reaches final bounce location in which case:
            - its distance from arm base is within reach of arm
            - OR its next deflection_y is beyond the table bounds
    :param deflections: list of tuples that contain the (x, y, vx, vy, time) of
    deflection
    """
    p_copy = copy.deepcopy(puck_pose)
    assert(0 <= p_copy.x <= table.width) 
    reach_radius = arm.link_length * arm.num_links
    dist_from_base = util.distance(arm.x, arm.y, p_copy.x, p_copy.y)
    if dist_from_base <= reach_radius:
        return deflections
    # (vx < MIN_V) implies puck moving straight down, no wall deflections
    # (vy < MIN_V) implies puck moving too slowly
    if abs(p_copy.vx) < MIN_V or abs(p_copy.vy) < MIN_V:  
        return deflections

    if len(deflections) > 0: prev_time = deflections[-1][4]
    else: prev_time = 0

    try:
        collision = vector_circle_intersect(arm, puck_pose, radius_prop)
        deflections.append([collision.x, collision.y,
                            puck_pose.vx, puck_pose.vy,
                            prev_time + collision.time_to_collision])
        return deflections
    
    except util.NoSolutionError:
        if p_copy.vx > 0:  # moving right
            time_deflection = (table.width - p_copy.x) / p_copy.vx
            p_copy.x = table.width  # next x position right after deflection
        else:  # moving left
            time_deflection = (0 - p_copy.x) / p_copy.vx
            p_copy.x = 0

        p_copy.vx *= -1
        p_copy.y = p_copy.y + p_copy.vy * time_deflection
        deflections.append([p_copy.x, p_copy.y,
                        p_copy.vx, p_copy.vy,
                        prev_time + time_deflection])
        return find_deflections(table, arm, p_copy, radius_prop, deflections)


def linearize_trajectory(puck_pose, deflections):
    if len(deflections) == 0:
        # no need for transformations, current pose will guide puck to arms
        return copy.deepcopy(puck_pose)
    else:
        # use last collision and treat as if constant velocity entire time
        # (x, y, vx, vy, time to reach position)
        new_p = util.Struct()
        last_collision = deflections[-1]
        last_x, last_y = last_collision[0], last_collision[1]
        last_vx, last_vy = last_collision[2], last_collision[3]
        total_time = last_collision[4]
        # x_0 = x_f - vx * t,  solve for linear trajectory with last velocity
        new_p.x = last_x - last_vx * total_time
        new_p.y = last_y - last_vy * total_time
        new_p.vx, new_p.vy = last_vx, last_vy
        return new_p


def linearize_trajectory_v1(table_length, table_width, puck_pose, extent_of_check):
    """
    NOTE: DO NOT USE THIS FUNCTION. DOES NOT WORK. Simply historical reference.
    Determine the collisions of the puck with the sides and find the final
    velocity and fake position of the puck as if the puck were only moving in
    a straight line at that velocity at same distance as currently is.
    :param puck_pose: contains info about orig puck's position and velocity
    :type: Struct()DEG_TO_RAD
    :param extent_of_check: scaling factor that scales the length of the table
    down and checks for all x-collisions up until that point. For example,
    maybe 1/2 of table length because after that the arms can hopefully reach,
    simplifying both arms' perimeters to straight line across table. NOTE: This
    is with robot side as the start of table, so 1/4 * length gives quarter way
    of table starting from robot side.
    :type: float
    :returns: (x_f, y_f, vx_f, vy_f, time_to_reach)
    After filtering out collisions with walls,
    this is the approximate pose of the puck as if it were only moving in a
    straight line from current position ignoring walls.
    """
    # should already be positive b/c negative velocity, but use abs() for safety
    time_to_reach = ((extent_of_check * table_length - puck_pose.y) /
                     (puck_pose.vy))
    total_x_dist = puck_pose.x + puck_pose.vx * time_to_reach
    num_wall_collisions = total_x_dist // table_width
    collisions = find_collisions(table_width, table_length,
                                 copy.deepcopy(puck_pose),
                                 num_wall_collisions)
    if (num_wall_collisions % 2 == 0):
        x_f = total_x_dist - num_wall_collisions * table_width
        vx_f = puck_pose.vx
    else:
        x_f = table_width - (total_x_dist - num_wall_collisions * table_width)
        vx_f = -puck_pose.vx
    # check to ensure math is all correct: puck should reach inside table
    # when in range of arms
    # assert(0 <= total_x_dist + vx_f * time_to_reach <= table_width)
    return (total_x_dist, puck_pose.y, vx_f, puck_pose.vy, time_to_reach)

