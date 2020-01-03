
import sim
import numpy as np

class ForceSensor:
    def __init__(self, clientID_arg, handle_arg):
        """
        ARGUMENTS:
        + clientID_arg: The clientID assigned by the simulation; expects type returned from sim.simxStart(...)
        + handle_arg: The handle of the force sensor object in the simulation;
        expects type returned from sim.simxGetObjectHandle(...)
        """
        self.clientID = clientID_arg
        self.handle = handle_arg
        self.force_vec = np.zeros(3)
        self.torque_vec = np.zeros(3)
    def __str__(self):
        print("force_vec: %s | torque_vec: %s" %(str(np.around(force_vec, 3)), str(np.around(torque_vec, 3))))
    def __repr__(self):
        print("An instance of the custom ForceSensor class")

    def update(self):
        """
        Grabs the values from the force sensor in the simulation and registers them into the properties
        self.force_vec and self.torque_vec.
        """
        return_code, state, self.force_vec, self.torque_vec = sim.simxReadForceSensor(self.clientID, self.handle, sim.simx_opmode_streaming)
        if (state >= 3):
            print("WARNING: Force sensor has been broken!")