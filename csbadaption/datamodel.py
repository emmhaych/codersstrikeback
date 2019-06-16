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
        self.map = [self.next_checkpoint]
        self.race_state = RaceState.UNKNOWN
        self.boost_used = False
        self.boost_location = None

        if previous_states:
            self.tick = previous_states.tick + 1
            self.vel = self.position - previous_states.position
            self.acc = self.vel - previous_states.vel
            self.map = previous_states.map
            self.race_state = previous_states.race_state
            self.boost_used = previous_states.boost_used
            self.boost_location = previous_states.boost_location
        else:
            self.race_state = RaceState.LAP_DISCOVERY

    def __str__(self):
        return str(self.__class__) + ': ' + str(self.__dict__)


class DataModel:
    def __init__(self):
        self.states_t = None
        self.states_tm1 = None

    def update(self, inputs):
        self.states_tm1 = self.states_t
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

            if self.states_tm1.race_state == RaceState.LAP_DISCOVERY:
                if len(self.states_t.map) > 1 and np.array_equal(self.states_t.next_checkpoint, self.states_t.map[0]):
                    self.states_t.race_state = RaceState.LAP_NEW
                elif not np.array_equal(self.states_t.next_checkpoint, self.states_t.map[-1]):
                    self.states_t.map.append(self.states_t.next_checkpoint)
            elif self.states_tm1.race_state == RaceState.LAP_NEW:
                self.states_t.race_state = RaceState.LAP_IN_PROGRESS
            elif self.states_tm1.race_state == RaceState.LAP_IN_PROGRESS:
                if not np.array_equal(self.states_tm1.next_checkpoint, self.states_t.next_checkpoint):
                    if np.array_equal(self.states_t.map[0], self.states_t.next_checkpoint):
                        self.states_t.race_state = RaceState.LAP_NEW

    def __str__(self):
        return str(self.__class__) + ': ' + \
            '; '.join(['{0}:{1}'.format(
                key, str(self.__dict__[key])) for key in self.__dict__.keys()]
            )
