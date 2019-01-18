# This file contains various utility functions with generic uses.

class Struct(object):
    """
    Generic class to store any variables, preferred over dictionary because of
    tab-completion capabilities.
    """
    pass


class NoSolutionError(Exception):
    """Error when trying to solve for intersection between puck trajectory
    and perimeter of arm reach."""
    pass


class MissingParamError(Exception):
    """
    Error when default parameter for a function is missing.
    """
    pass


def distance(x1, y1, x2, y2):
    return ((x1 - x2)**2 + (y1 - y2)**2)**0.5


def transform(value, is_pos, y_bound=None):
    """
    Converts between graphics and cartesian coordinate frame. In graphics frame,
    y = 0 at the top, but in cartesian, y = 0 at bottom. Velocities are
    reversed.

    :param value: value to be transformed
    :type: float

    :param y_bound: max value of y, used to transform position y values
    :type: float or int

    :param is_pos: is the value to be converted a position or velocity. If
    position, flip frame by subtracting from max y value. If velocity, simply
    reverse direction.

    """
    if is_pos:
        try:
            return y_bound - value
        except TypeError:
            raise MissingParamError('Missing y_bound for position transform')
    else: return -1 * value


def pass_vel_threshold(vel_y, table_length, table_proportion):
    """
    Checks if puck's velocity y-component is above a certain magnitude threshold. If y-velocity too low,
    will actually cause recursion stack overflow. Note: Please make sure to normalize the velocity with
    some reasonable time unit like seconds. If you use milliseconds or smaller, this threshold will not work.

    """
    assert(0 < table_proportion < 1)
    return abs(vel_y) > table_proportion * table_length


def past_table_bounds(puck_y, table_length):
    """
    Checks if puck's y position is out of range of the table. Of course this won't happen ever, but
    this checks when the puck should have intersected with either length-wise end of the table.
    """
    return 0 <= puck_y <= table_length


def is_puck_approaching(puck_vy):
    """
    Seemingly simple, but important check so avoid doing arm-puck collision calculations
    if puck isn't coming to arm. Also avoids other unforeseen problems with calculations.

    NOTE: This is defined with the cartesian coordinate system where the robot arm is located at y = 0,
    so the puck is only moving toward the arm if its velocity is negative.

    """
    return puck_vy < 0
