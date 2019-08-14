import math

import motion_planning as m
import utilities as util
"""Extensive test-cases for computations of arm trajectory motion.
Trying to debug with a simulator only introduces additional
bugs that make it hard to understand if the bug is in computation 
or in the simulator.
"""

# we define left edge of table as x = 0
# arm_x = table center
# arm_y = 0
arm = util.Struct()
arm.x, arm.y = 1., 0.
arm.num_links = 1
arm.link_length = 1
radius_prop = 1

table = util.Struct()
table.width = 2    # x
table.length = 10  # y

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
    assert(math.isclose(new_x, x, rel_tol=1e-7))
    assert(math.isclose(new_y, y, rel_tol=1e-7))
    return


def test_vector_circle_intersect():
    # basic case
    puck = util.Struct()
    puck.x = -99
    puck.y = 100
    puck.vx = 1
    puck.vy = -1
    collision = m.vector_circle_intersect(arm, puck, radius_prop)
    exp_x, exp_y = 1-0.5*(2**0.5), 0.5*(2**0.5)  # 1-sqrt(2)/2, sqrt(2)/2
    # t = (yf - yi) / vy
    exp_time = (exp_y - puck.y) / puck.vy
    assert(math.isclose(collision.x, exp_x, rel_tol=1e-7))
    assert(math.isclose(collision.y, exp_y, rel_tol=1e-7))
    assert(math.isclose(collision.time_to_collision, exp_time, rel_tol=1e-7))

    # edge case, puck hits very edge
    puck.x = 3
    puck.y = 1
    puck.vx = -1
    puck.vy = -1
    collision = m.vector_circle_intersect(arm, puck, radius_prop)
    exp_x, exp_y = 2, 0
    exp_time = 1
    assert(math.isclose(collision.x, exp_x, rel_tol=1e-7))
    assert(math.isclose(collision.y, exp_y, rel_tol=1e-7))
    assert(math.isclose(collision.time_to_collision, exp_time, rel_tol=1e-7))

    # Test assertion, ball fails to reach y >= 0
    # edge case, puck hits very edge
    puck.x = 4
    puck.y = 1
    raised_assertion = False
    try:
        collision = m.vector_circle_intersect(arm, puck, radius_prop)
    except util.NoSolutionError:
        raised_assertion = True
    
    assert(raised_assertion)
    return


def test_find_deflections():
    puck = util.Struct()
    puck.x = 0
    puck.y = 10
    puck.vx = 1
    puck.vy = -1

    deflections = m.find_deflections(table, arm, puck,
            radius_prop)
    assert(deflections == [
        # [x, y, vx, vy, t]
        [2, 8, -1, -1, 2],
        [0, 6, 1, -1, 4],
        [2, 4, -1, -1, 6],
        [0, 2, 1, -1, 8],
        [1, 1, 1, -1, 9]  # intersect with radius of arm
    ])

    puck.x = 2
    puck.y = 6
    puck.vx = 1
    puck.vy = -2.5
    # puck starts off immediately colliding with table
    deflections = m.find_deflections(table, arm, puck,
            radius_prop)
    assert(deflections[:2] == [
        # [x, y, vx, vy, t]
        [2, 6, -1, -2.5, 0],  # immediately collide
        [0, 1, 1, -2.5, 2]    # test first two collisions only
    ])
    return

    # puck moves straight down, no deflections, but reaches arm directly
    puck.x = 0
    puck.y = 4
    puck.vx = 1
    puck.vy = -2
    # puck starts off immediately colliding with table
    deflections = m.find_deflections(table, arm, puck,
            radius_prop)
    assert(len(deflections) == 1)
    # intersect with arm somewhere between table edges
    assert(0 < deflections[0] < table.width)
    assert(0 < deflections[1])
    assert(deflections[2] == 1)  # puck still moving at vx = 1

def test_linearize_trajectory():

    return


test_vector_circle_intersect()
test_calc_joints_from_pos()
test_find_deflections()
test_linearize_trajectory()
test_predict_puck_motion()
