
import sim
import numpy as np
import transforms3d

from InertialMeasurementUnit import InertialMeasurementUnit as Imu

if __name__ == "__main__":
    ### BEGIN SIM CONNECTION ###
    connection_successful = False
    print("=== Programme START ===")
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart("127.0.0.1",19999,True,True,5000,5) # Connect to CoppeliaSim
    if clientID!=-1:
        print ("Connected to remote API server")
        connection_successful = True
    else:
        print ("Failed connecting to remote API server")
        connection_successful = False

    if connection_successful:
        ### GET OBJECT HANDLES ###
        return_code, world_frame = sim.simxGetObjectHandle(clientID, "world_frame", sim.simx_opmode_blocking)
        return_code, cube = sim.simxGetObjectHandle(clientID, "cube", sim.simx_opmode_blocking)
        ### DO STUFF ###
        imu = Imu(clientID, cube, world_frame)
        while True:
            imu.update()
            print(imu)

        ### CLOSE CONNECTION TO SIM ###
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    print("=== Programme end ===")