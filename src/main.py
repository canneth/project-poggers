
import sim
import numpy as np
import transforms3d
import time

from Pog2 import Pog2

if __name__ == "__main__":
    ### BEGIN SIM CONNECTION ###
    connection_successful = False
    print("=== Programme START ===")
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID = sim.simxStart("127.0.0.1",19997,True,True,5000,5) # Connect to CoppeliaSim
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
        return_code, hip_joint_x = sim.simxGetObjectHandle(clientID, "hip_joint_x", sim.simx_opmode_blocking)
        return_code, hip = sim.simxGetObjectHandle(clientID, "hip", sim.simx_opmode_blocking)
        return_code, hip_joint_y = sim.simxGetObjectHandle(clientID, "hip_joint_y", sim.simx_opmode_blocking)
        return_code, leg_actuator_anchor_body_x = sim.simxGetObjectHandle(clientID, "leg_actuator_anchor_body_x", sim.simx_opmode_blocking)
        return_code, leg_actuator_anchor_body_y = sim.simxGetObjectHandle(clientID, "leg_actuator_anchor_body_y", sim.simx_opmode_blocking)
        return_code, leg_actuator_x = sim.simxGetObjectHandle(clientID, "leg_actuator_x", sim.simx_opmode_blocking)
        return_code, leg_actuator_y = sim.simxGetObjectHandle(clientID, "leg_actuator_y", sim.simx_opmode_blocking)
        return_code, leg_actuator_anchor_leg_x = sim.simxGetObjectHandle(clientID, "leg_actuator_anchor_leg_x", sim.simx_opmode_blocking)
        return_code, leg_actuator_anchor_leg_y = sim.simxGetObjectHandle(clientID, "leg_actuator_anchor_leg_y", sim.simx_opmode_blocking)
        return_code, leg_cylinder = sim.simxGetObjectHandle(clientID, "leg_cylinder", sim.simx_opmode_blocking)
        return_code, leg_actuator_extension = sim.simxGetObjectHandle(clientID, "leg_actuator_extension", sim.simx_opmode_blocking)
        return_code, leg_piston = sim.simxGetObjectHandle(clientID, "leg_piston", sim.simx_opmode_blocking)
        return_code, foot_force_sensor = sim.simxGetObjectHandle(clientID, "foot_force_sensor", sim.simx_opmode_blocking)
        return_code, foot = sim.simxGetObjectHandle(clientID, "foot", sim.simx_opmode_blocking)

        ### SETUP ###
        pog = Pog2(
            clientID,
            world_frame,
            body,
            hip_joint_x,
            hip,
            hip_joint_y,
            leg_actuator_anchor_body_x,
            leg_actuator_anchor_body_y,
            leg_actuator_x,
            leg_actuator_y,
            leg_actuator_anchor_leg_x,
            leg_actuator_anchor_leg_y,
            leg_cylinder,
            leg_actuator_extension,
            leg_piston,
            foot_force_sensor,
            foot,
            debug_flag = False
        )

        print("Setup done, entering while loop...")

        ### LOOP ###
        while True:
            pog.simStep()
            pass


        ### CLOSE CONNECTION TO SIM ###
        sim.simxGetPingTime(clientID)
        sim.simxFinish(clientID)
    print("=== Programme end ===")