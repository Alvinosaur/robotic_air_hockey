import math

import motion_planning as m
"""Extensive test-cases for computations of arm trajectory motion.
Trying to debug with a simulator only introduces additional
bugs that make it hard to understand if the bug is in computation 
or in the simulator.
"""

def test_predict_puck_motion():
    
    return

def test_calc_joints_from_pos():
    arm_length = 4
    # test straight up
    x, y = 0, 8
    theta0, theta1 = math.pi/2, 0
    assert(m.calc_joints_from_pos(arm_length, x, y) == 
        (theta0, theta1))

    # test elbow doesn't reach below x-axis since would run into table edge
    x, y = 3, -1
    theta0, theta1 = m.calc_joints_from_pos(arm_length, x, y)
    new_x = arm_length * math.cos(theta0) + (
        arm_length * math.cos(theta0 + theta1)
    )
    new_y = arm_length * math.sin(theta0) + (
        arm_length * math.sin(theta0 + theta1)
    )
    assert(0 <= theta0 and theta0 <= math.pi)
    assert(new_x == x, new_y == y)
    return

def test_vector_circle_intersect():

    return

def test_vector_circle_intersect():

    return

def test_find_deflections():

    return

def test_linearize_trajectory():

    return


test_calc_joints_from_pos()
test_find_deflections()
test_linearize_trajectory()
test_predict_puck_motion()
test_vector_circle_intersect()