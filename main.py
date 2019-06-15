import sys
import math
import numpy as np
from enum import Enum

# Setup States
input_states = [
    'pos',
    'next_checkpoint_pos',
    'next_checkpoint_dist',
    'next_checkpoint_angle',
    'opp_pos'
    ]
remembered_states = [
    'prev_checkpoint_pos'
]
computed_states = [
    'tick',
    'vel',
    'acc',
    'pos_error',
    'map',
    'race_state'
]
all_states = input_states + computed_states + remembered_states

states = dict((el, None) for el in (all_states))
previous_states = None


class RaceState(Enum):
    START = 0
    LAP_DISCOVERY = 1
    LAP_NEW = 2
    LAP_IN_PROGRESS = 3


def get_inputs():
    # Parse input_states
    x, y, next_checkpoint_x, next_checkpoint_y, next_checkpoint_dist, next_checkpoint_angle = [int(i) for i in input().split()]
    opp_x, opp_y = [int(i) for i in input().split()]

    states['pos'] = np.array([x, -1 * y], 'i4')
    states['next_checkpoint_pos'] = np.array([next_checkpoint_x, -1 * next_checkpoint_y], 'i4')
    states['next_checkpoint_dist'] = next_checkpoint_dist
    states['next_checkpoint_angle'] = next_checkpoint_angle
    states['opp_pos'] = np.array([opp_x, -1 * opp_y], 'i4')


def compute_states():
    # These computed states require historic information
    if previous_states['tick'] != None:
        states['tick'] = previous_states['tick'] + 1
        states['vel'] = states['pos'] - previous_states['pos']
        states['acc'] = states['vel'] - previous_states['vel']
        states['map'] = previous_states['map']
        states['race_state'] = previous_states['race_state']
        
        if not np.array_equal(previous_states['next_checkpoint_pos'], states['next_checkpoint_pos']):
            # Just crossed checkpoint
            states['prev_checkpoint_pos'] = previous_states['next_checkpoint_pos']
            """
            if np.equal(states['next_checkpoint_pos'], states['map'][0]):
                states['race_state'] = RaceState.LAP_NEW
            elif states['race_state'] == RaceState.LAP_DISCOVERY:
                states['map'].append(states['next_checkpoint_pos'])
            """
        else:
            states['prev_checkpoint_pos'] = previous_states['prev_checkpoint_pos']

        # Race state is discovery until the same checkpoint is recorded
        if states['race_state'] == RaceState.START:
            states['race_state'] = RaceState.LAP_DISCOVERY
        elif states['race_state'] == RaceState.LAP_NEW:
            states['race_state'] = RaceState.LAP_IN_PROGRESS
    else:
        states['tick'] = 0
        states['vel'] = 0
        states['acc'] = 0
        previous_states['pos_error'] = np.array([0, 0])
        states['prev_checkpoint_pos'] = states['pos']
        states['map'] = [states['next_checkpoint_pos']]
        states['race_state'] = RaceState.START


def send_command(bearing, thrust):
    # Bearing (in degrees) is used to calculate target_x and target_y at a set radius from current position
    # target_x and target_y are related and bearing simplifies the output
    radius = 100    # pixels
    direction_vector = np.array([math.sin(math.radians(bearing)), math.cos(math.radians(bearing))], 'f4')
    command_coordinate = states['pos'] + (radius * direction_vector)
    print('{0} {1} {2}'.format(int(round(command_coordinate[0])), int(round(-1 * command_coordinate[1])), thrust))


def get_command_bearing():
    trajectory_vector = states['next_checkpoint_pos'] - states['pos']
    error_vector = np.array([trajectory_vector[1], -1 * trajectory_vector[0]])
    
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

    trajectory_uv = (states['next_checkpoint_pos'] - states['prev_checkpoint_pos']).astype('f4')
    trajectory_uv /= np.linalg.norm(trajectory_uv)
    ideal_position = states['prev_checkpoint_pos'] + np.dot(states['pos'] - states['prev_checkpoint_pos'], trajectory_uv) * trajectory_uv
    states['pos_error'] = states['pos'] - ideal_position
    
    vector_to_checkpoint = states['next_checkpoint_pos'] - states['pos']
    trajectory_bearing = math.degrees(math.atan2(vector_to_checkpoint[0], vector_to_checkpoint[1])) % 360
    
    # Get errors
    if np.cross(states['pos_error'], trajectory_uv) > 0:
        error_sign = -1
    else:
        error_sign = 1
    error_p = error_sign * np.linalg.norm(states['pos_error'])
    error_d = error_sign * np.linalg.norm(previous_states['pos_error']) - error_sign * np.linalg.norm(states['pos_error'])
    
    # PD controller
    P = 0.005
    D = 0
    result = P * error_p + D * error_d
    
    if result > 180:
        result = 180
    elif result < -180:
        result = -180
    
    return (trajectory_bearing + result) % 360


def get_command_thrust():
    turning_thrshold = 30
    if (
            (states['next_checkpoint_angle'] > turning_thrshold) or 
            (states['next_checkpoint_angle'] < (-1 * turning_thrshold)) or 
            states['next_checkpoint_dist'] < 600
        ):
        thrust = 0
    else:
        thrust = 100

    return thrust


if __name__ == '__main__':
    while True:
        # Refresh states
        previous_states = states
        states = dict((el, None) for el in (all_states))

        # Parse inputs and compute new states
        get_inputs()
        compute_states()
        print(states, file=sys.stderr)

        # Compute command
        command_bearing = get_command_bearing()
        print(command_bearing, file=sys.stderr)
        command_thurst = get_command_thrust()
        
        # Send command
        send_command(command_bearing, command_thurst)

        previous_states = states
