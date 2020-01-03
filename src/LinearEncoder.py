
import sim
import numpy as np

class LinearEncoder:
    def __init__(self, clientID_arg, from_handle_arg, to_handle_arg):
        """
        ARGUMENTS:
        + clientID_arg: The clientID assigned by the simulation; expects type returned from sim.simxStart(...)
        + from_handle_arg: The handle of the object wrt which the linear encoder extends;
        expects type returned from sim.simxGetObjectHandle(...)
        + to_handle_arg: The handle of the object to which the linear encoder extends;
        expects type returned from sim.simxGetObjectHandle(...)
        """
        self.clientID = clientID_arg
        self.from_handle = from_handle_arg
        self.to_handle = to_handle_arg
        self.distance = None
    def __str__(self):
        print(self.distance)
    def __repr__(self):
        print("An instance of the custom LinearEncoder class")

    def update(self):
        """
        Grabs the distance from from_handle to to_handle and registers it into
        the property self.distance.
        """
        return_code, pos_new = sim.simxGetObjectPosition(self.clientID, self.from_handle, self.to_handle, sim.simx_opmode_streaming)
        self.distance = np.linalg.norm(np.asarray(pos_new))
        