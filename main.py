from csbadaption.engine import Engine
from csbadaption.datamodel import RaceState
import numpy as np
import math
import sys


def get_command_bearing(datamodel):
    datamodel.states_t.trajectory = \
        datamodel.states_t.next_checkpoint - datamodel.states_t.position
    error_vector = np.array([
        datamodel.states_t.trajectory[1],
        -1 * datamodel.states_t.trajectory[0]
    ])

    # Source: https://stackoverflow.com/a/5227626

    # P - point
    # D - direction of line (unit length)
    # A - point in line
    # X - base of the perpendicular line

    #     P
    #    /|
    #   / |
    #  /  v
    # A---X----->D

    # (P-A).D == |X-A|

    # X == A + ((P-A).D)D
    # Desired perpendicular: X-P

    trajectory_uv = (
        datamodel.states_t.next_checkpoint -
        datamodel.states_t.prev_checkpoint
    ).astype('f4')
    trajectory_uv /= np.linalg.norm(trajectory_uv)
    ideal_position = datamodel.states_t.prev_checkpoint + np.dot(
        datamodel.states_t.position - datamodel.states_t.prev_checkpoint,
        trajectory_uv
    ) * trajectory_uv
    datamodel.states_t.pos_error = datamodel.states_t.position - ideal_position

    vector_to_checkpoint = \
        datamodel.states_t.next_checkpoint - datamodel.states_t.position
    trajectory_bearing = math.degrees(math.atan2(
        vector_to_checkpoint[0],
        vector_to_checkpoint[1])
    ) % 360

    # Get errors
    if np.cross(datamodel.states_t.pos_error, trajectory_uv) > 0:
        error_sign = -1
    else:
        error_sign = 1
    error_p = error_sign * np.linalg.norm(datamodel.states_t.pos_error)
    error_d = error_sign * np.linalg.norm(datamodel.states_t.pos_error) \
        - error_sign * np.linalg.norm(datamodel.states_t.pos_error)

    # PD controller
    P = 0.005
    D = 0
    result = P * error_p + D * error_d

    if result > 180:
        result = 180
    elif result < -180:
        result = -180

    return (trajectory_bearing + result) % 360


def get_command_thrust(datamodel):
    turning_thrshold = 30
    if (
        (datamodel.states_t.next_checkpoint_angle > turning_thrshold) or
        (datamodel.states_t.next_checkpoint_angle < (-1 * turning_thrshold)) or
        datamodel.states_t.next_checkpoint_dist < 600
    ):
        thrust = 0
    else:
        thrust = 100

    return thrust


def bt_to_xyt(datamodel, bearing, thrust):
    # Bearing (in degrees) is used to calculate target_x and target_y at a
    # set radius from current position
    # target_x and target_y are related and bearing simplifies the output
    radius = 100    # pixels
    direction_vector = np.array(
        [math.sin(math.radians(bearing)), math.cos(math.radians(bearing))],
        np.float
    )
    command_coordinate = datamodel.states_t.position + \
        (radius * direction_vector)
    x = int(round(command_coordinate[0]))
    y = int(round(-1 * command_coordinate[1]))
    return x, y, thrust


def determine_boost_location(datamodel):
    distances = []
    for x, y in \
        [[i, (i + 1) % len(datamodel.states_t.map)]
            for i in range(len(datamodel.states_t.map))]:
        distances.append(
            np.linalg.norm(
                datamodel.states_t.map[x] - datamodel.states_t.map[y]
            )
        )
    print('Distances: {0}'.format(distances), file=sys.stderr)
    return np.argmax(distances)


def tick(datamodel):
    bearing = get_command_bearing(datamodel)
    thrust = get_command_thrust(datamodel)

    if datamodel.states_t.tick > 0:
        if datamodel.states_tm1.race_state == RaceState.LAP_DISCOVERY and \
                datamodel.states_t.race_state == RaceState.LAP_NEW:
            datamodel.states_t.boost_location = determine_boost_location(datamodel)

    if datamodel.states_t.boost_location and not datamodel.states_t.boost_used:
        if np.array_equal(datamodel.states_tm1.prev_checkpoint, datamodel.states_t.map[datamodel.states_t.boost_location]):
            if abs(datamodel.states_t.next_checkpoint_angle) < 15:
                thrust = 'BOOST'
                datamodel.states_t.boost_used = True

    return bt_to_xyt(datamodel, bearing, thrust)


print('Hello, World', file=sys.stderr)
gameengine = Engine(tick)
gameengine.start()
