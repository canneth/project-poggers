
import sim
import numpy as np
import transforms3d

from InertialMeasurementUnit import InertialMeasurementUnit as Imu
from LinearEncoder import LinearEncoder
from ForceSensor import ForceSensor

SUPPORT_PHASE = 1
FLIGHT_PHASE = 2

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
        return_code, body = sim.simxGetObjectHandle(clientID, "body", sim.simx_opmode_blocking)
        return_code, hip_joint = sim.simxGetObjectHandle(clientID, "hip_joint", sim.simx_opmode_blocking)
        return_code, leg_cylinder = sim.simxGetObjectHandle(clientID, "leg_cylinder", sim.simx_opmode_blocking)
        return_code, leg_joint = sim.simxGetObjectHandle(clientID, "leg_joint", sim.simx_opmode_blocking)
        return_code, leg_piston = sim.simxGetObjectHandle(clientID, "leg_piston", sim.simx_opmode_blocking)
        return_code, foot_force_sensor = sim.simxGetObjectHandle(clientID, "foot_force_sensor", sim.simx_opmode_blocking)
        return_code, foot = sim.simxGetObjectHandle(clientID, "foot", sim.simx_opmode_blocking)

        ### DO STUFF ###
        imu = Imu(clientID, body, world_frame)
        leg_encoder = LinearEncoder(clientID, leg_cylinder, leg_piston)
        foot_sensor = ForceSensor(clientID, foot_force_sensor)

        leg_extension = None
        leg_extension_dot = None
        leg_extension_dot_new = None
        leg_extension_dotdot = -10

        while True:
            foot_sensor.update()
            foot_force = np.linalg.norm(foot_sensor.force_vec)
            collision = False
            if (foot_force >= 1.0):
                collision = True
            else:
                collision = False
            if collision:
                # Check for nadir of support phase
                leg_encoder.update()
                leg_extension_new = leg_encoder.distance
                if leg_extension == None:
                    leg_extension = leg_extension_new
                if (leg_extension_new != leg_extension):
                    leg_extension_dot_new = leg_extension_new - leg_extension
                    if leg_extension_dot == None:
                        leg_extension_dot = leg_extension_dot_new
                    if (leg_extension_dot_new != leg_extension_dot):
                        leg_extension_dotdot = leg_extension_dot_new - leg_extension_dot
                        leg_extension_dot = leg_extension_dot_new
                    leg_extension = leg_extension_new
                if (leg_extension_dotdot >= 0 and leg_extension_dot >= 0):
                    # At nadir of support phase, extend leg
                    print("EXTEND leg")
                    sim.simxSetJointTargetPosition(clientID, leg_joint, 0.05, sim.simx_opmode_streaming)
            else:
                # Retract leg
                print("RETRACT leg")
                sim.simxSetJointTargetPosition(clientID, leg_joint, 0.00, sim.simx_opmode_streaming)

        ### CLOSE CONNECTION TO SIM ###
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    print("=== Programme end ===")