import numpy as np
from enum import Enum


class RaceState(Enum):
    UNKNOWN = 0
    LAP_DISCOVERY = 1
    LAP_NEW = 2
    LAP_IN_PROGRESS = 3


class States:
    def __init__(self, inputs, previous_states):
        self.position = np.array(
            [inputs.x, -1 * inputs.y],
            np.int32
        )
        self.next_checkpoint = np.array(
            [inputs.next_checkpoint_x, -1 * inputs.next_checkpoint_y],
            np.int32
        )
        self.next_checkpoint_dist = inputs.next_checkpoint_dist
        self.next_checkpoint_angle = inputs.next_checkpoint_angle
        self.opp_position = np.array(
            [inputs.opp_x, -1 * inputs.opp_y],
            np.int32
        )

        self.tick = 0
        self.vel = 0
        self.acc = 0
        self.pos_error = 0
        self.prev_checkpoint = self.position
        self.trajectory = None

        if previous_states:
            self.tick = previous_states.tick + 1
            self.vel = self.position - previous_states.position
            self.acc = self.vel - previous_states.vel

    def __str__(self):
        return str(self.__class__) + ': ' + str(self.__dict__)


class DataModel:
    def __init__(self):
        self.states_t = None
        self.states_tm1 = None

    def update(self, inputs):
        self.states_t = States(inputs, self.states_tm1)

        if self.states_tm1:
            if not np.array_equal(
                self.states_tm1.next_checkpoint,
                self.states_t.next_checkpoint
            ):
                self.states_t.prev_checkpoint = \
                    self.states_tm1.next_checkpoint
            else:
                self.states_t.prev_checkpoint = \
                    self.states_tm1.prev_checkpoint

        self.states_tm1 = self.states_t

    def __str__(self):
        return str(self.__class__) + ': ' + \
            '; '.join(['{0}:{1}'.format(
                key, str(self.__dict__[key])) for key in self.__dict__.keys()]
            )
