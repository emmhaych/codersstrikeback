import sys
from csbadaption.io import IO
from csbadaption.datamodel import DataModel


class Engine:
    def __init__(self, user_function):
        self.datamodel = DataModel()
        self.user_function = user_function
        print('Initalized game engine', file=sys.stderr)

    def tick(self):
        print('Start of Frame', file=sys.stderr)
        inputs = IO.parse_inputs()
        self.datamodel.update(inputs)

        print(self.datamodel, file=sys.stderr)

        # Run user code
        x, y, thrust = self.user_function(self.datamodel)

        # Send command
        IO.send_raw_outputs(x, y, thrust)
        
        print('End of Frame', file=sys.stderr)

    def start(self):
        while True:
            self.tick()
