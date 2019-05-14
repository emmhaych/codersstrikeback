import sys
import math
import numpy as np

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
    'pos_err'
]
all_states = input_states + computed_states + remembered_states

states = dict((el, None) for el in (all_states))
previous_states = None


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
        
        if not np.array_equal(previous_states['next_checkpoint_pos'], states['next_checkpoint_pos']):
            states['prev_checkpoint_pos'] = previous_states['next_checkpoint_pos']
        else:
            states['prev_checkpoint_pos'] = previous_states['prev_checkpoint_pos']
    else:
        states['tick'] = 0
        states['vel'] = 0
        states['acc'] = 0
        previous_states['pos_err'] = np.array([0, 0])
        states['prev_checkpoint_pos'] = states['pos']


def send_command(bearing, thrust):
    # Bearing (in degrees) is used to calculate target_x and target_y at a set radius from current position
    # target_x and target_y are related and bearing simplifies the output
    radius = 100    # pixels
    direction_vector = np.array([math.sin(math.radians(bearing)), math.cos(math.radians(bearing))], 'f4')
    command_coordinate = states['pos'] + (radius * direction_vector)
    print([bearing, direction_vector, command_coordinate], file=sys.stderr)
    print('{0} {1} {2}'.format(int(round(command_coordinate[0])), int(round(-1 * command_coordinate[1])), thrust))


if __name__ == '__main__':
    while True:
        # Refresh states
        previous_states = states
        states = dict((el, None) for el in (all_states))

        # Parse inputs and compute new states
        get_inputs()
        compute_states()
        print(states, file=sys.stderr)
        
        # Send command
        command_vector = states['next_checkpoint_pos'] - states['pos']
        bearing = math.degrees(math.atan2(command_vector[0], command_vector[1])) % 360
        thrust = 100
        send_command(bearing, thrust)

        previous_states = states