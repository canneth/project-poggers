
import sim
import numpy as np
import transforms3d

class InertialMeasurementUnit:
    def __init__(self, clientID_arg, handle_arg, world_frame_handle_arg):
        """
        ARGUMENTS:
        + clientID_arg: The clientID assigned by the simulation; expects type returned from sim.simxStart(...)
        + handle_arg: The handle of the object to be used as the IMU in the simulation;
        expects type returned from sim.simxGetObjectHandle(...)
        + world_frame_handle_arg: The handle of the world_frame wrt which the orientation of the IMU
        is to be determined; expects type returned from sim.simxGetObjectHandle(...)
        """
        self.clientID = clientID_arg
        self.handle = handle_arg
        self.world_frame = world_frame_handle_arg
        self.rot_mat = np.zeros((3, 3))
    def __str__(self):
        return str(np.around(self.rot_mat, 5))
    def __repr(self):
        return str("An instance of custom class IMU")
        

    def update(self):
        """
        Grabs the orientation of handle wrt world_frame and registers it into the x, y, z properties.
        To be called before reading IMU values.
        """
        return_success, euler_angles = sim.simxGetObjectOrientation(self.clientID, self.handle, self.world_frame, sim.simx_opmode_streaming)
        self.rot_mat = transforms3d.euler.euler2mat(euler_angles[0], euler_angles[1], euler_angles[2], axes = "rxyz")
