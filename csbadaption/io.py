from enum import Enum
import numpy as np


class RawInputs:
    def __init__(self, values=[None]*8):
        self.x = values[0]
        self.y = values[1]
        self.next_checkpoint_x = values[2]
        self.next_checkpoint_y = values[3]
        self.next_checkpoint_dist = values[4]
        self.next_checkpoint_angle = values[5]
        self.opp_x = values[6]
        self.opp_y = values[7]

    def __str__(self):
        return str(self.__class__) + ':' + str(self.__dict__)


class RawOutputs:
    def __init__(self, x, y, thrust):
        self.x = x
        self.y = y
        self.thrust = thrust

    def __str__(self):
        return '{0} {1} {2}'.format(self.x, self.y, self.thrust)


class IO:
    @staticmethod
    def parse_inputs():
        inputs = [int(i) for i in input().split()]
        inputs.extend([int(i) for i in input().split()])
        raw_inputs = RawInputs(inputs)

        return raw_inputs

    @staticmethod
    def send_raw_outputs(x, y, thrust):
        print(RawOutputs(x, y, thrust))
