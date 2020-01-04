
import sim

import numpy as np

class RotaryEncoder:
    def __init__(self, clientID_arg, handle_arg):
        """
        ARGUMENTS:
        + clientID_arg: The clientID assigned by the simulation; expects type returned from sim.simxStart(...)
        + handle_arg: The handle of the joint to attach the encoder to;
        expects type returned from sim.simxGetObjectHandle(...)
        """
        self.clientID = clientID_arg
        self.handle = handle_arg
        self.angle = 0
    def __str__(self):
        print(angle)
    def __repr__(self):
        print("An instance of the custom RotaryEncoder class")

    def update(self):
        """
        Grabs joint angle and registers it to the property self.angle.
        """
        return_code, angle = sim.simxGetJointPosition(self.clientID, self.handle, sim.simx_opmode_streaming)
        self.angle = angle