
import sim
import numpy as np
import transforms3d
import time

from InertialMeasurementUnit import InertialMeasurementUnit as Imu
from LinearEncoder import LinearEncoder
from ForceSensor import ForceSensor
from RotaryEncoder import RotaryEncoder

SUPPORT_PHASE = 1
TRANSFER_PHASE = 2

NEUTRAL_LEG_LENGTH = 0.15

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

        ### SENSORS ###
        imu = Imu(clientID, body, world_frame)
        leg_encoder = LinearEncoder(clientID, leg_cylinder, leg_piston)
        foot_sensor = ForceSensor(clientID, foot_force_sensor)
        hip_encoder = RotaryEncoder(clientID, hip_joint)

        leg_extension = None
        leg_extension_dot = None
        leg_extension_dot_new = None
        leg_extension_dotdot = -10

        hip_joint_angle = 0
        pep_pos = np.zeros(3)
        aep_pos = np.zeros(3)
        neutral_pos = np.zeros(3)

        phase = TRANSFER_PHASE
        support_start_time = None
        support_end_time = None
        support_phase_duration = 0
        cg_print = 0
        body_vel = 0

        support_start_time_grabbed = False
        support_end_time_grabbed = False
        pep_pos_grabbed = False

        pitch_angle_prev = 0

        transfer_start_time = 0
        transfer_start = True

        while True:
            foot_sensor.update()
            foot_force = np.linalg.norm(foot_sensor.force_vec)
            if (foot_force >= 5.0):
                phase = SUPPORT_PHASE
            else:
                phase = TRANSFER_PHASE
            
            ### HOP-HEIGHT CONTROLLER ###
            if (phase == SUPPORT_PHASE):
                # Check for nadir of support phase by using leg extension as heuristic
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
                    sim.simxSetJointTargetPosition(clientID, leg_joint, 0.05, sim.simx_opmode_streaming)
                transfer_start = False
            else:
                if not transfer_start:
                    transfer_start_time = time.time()
                    transfer_start = True
                if transfer_start:
                    transfer_time_elapsed = time.time() - transfer_start_time
                    if (transfer_time_elapsed >= 0.1):
                        # Return leg to neutral length
                        sim.simxSetJointTargetPosition(clientID, leg_joint, 0.00, sim.simx_opmode_streaming)
                    else:
                        # Retract leg completely
                        sim.simxSetJointTargetPosition(clientID, leg_joint, -0.05, sim.simx_opmode_streaming)

            ### VELOCITY CONTROLLER ###
            if (phase == SUPPORT_PHASE):
                # Start timer
                if not support_start_time_grabbed:
                    support_start_time = time.time()
                    support_start_time_grabbed = True
                    support_end_time_grabbed = False
                # Grab pep_pos
                if not pep_pos_grabbed:
                    return_code, pep_pos = sim.simxGetObjectPosition(clientID, body, foot, sim.simx_opmode_streaming)
                    pep_pos_grabbed = True
                # Read hip_joint angle (make and use RotaryEncoder class)
                hip_encoder.update()
                hip_joint_angle = hip_encoder.angle
                # Use hip_joint angle to grab aep_pos of foot (wrt body) [Cheating a lil' here]
                return_code, aep_pos = sim.simxGetObjectPosition(clientID, body, foot, sim.simx_opmode_streaming)
                aep_pos = np.asarray(aep_pos)
                # Set neutral_pos as -aep_pos
                neutral_pos = aep_pos
            else:
                # # Estimate body velocity from support phase duration and foot position
                # if not support_end_time_grabbed:
                #     support_end_time = time.time()
                #     support_end_time_grabbed = True
                #     support_start_time_grabbed = False
                #     if (support_start_time != None):
                #         support_phase_duration = support_end_time - support_start_time
                # if pep_pos_grabbed:
                #     cg_print = aep_pos[1] - pep_pos[1]
                # else:
                #     cg_print = 2*NEUTRAL_LEG_LENGTH*np.sin(hip_joint_angle)
                # if (support_phase_duration != 0):
                #     body_vel = -cg_print/support_phase_duration
                """
                Using the heuristic of cg_print/support_phase_duration to estimate
                body_vel causes system to oscillate out of control over time.
                I suspect it's because the errors in body_vel grow proportionately as
                the velocity error grows.

                Using the true body_vel derived from the simulator works like a charm.
                This lends credibility to the above suspicion.
                """
                return_code, linear_vel, angular_vel = sim.simxGetObjectVelocity(clientID, body, sim.simx_opmode_streaming)
                imu.update()
                vel_vec = np.dot(np.linalg.inv(imu.rot_mat), np.reshape(np.asarray(linear_vel), (3, 1)))
                body_vel = vel_vec[1]

                pep_pos_grabbed = False
                desired_vel = 0.03
                # vel_p_gain = 0.285 # Using body_vel obtained by cg_print heuristic
                vel_p_gain = 0.1 # Using true body_vel derived from simulator data
                foot_forward_distance = -vel_p_gain*(desired_vel - body_vel)
                if (abs(foot_forward_distance) >= NEUTRAL_LEG_LENGTH*0.99):
                    foot_forward_distance = NEUTRAL_LEG_LENGTH*0.99*(foot_forward_distance/abs(foot_forward_distance))
                hip_joint_angle_error = np.arcsin(foot_forward_distance/NEUTRAL_LEG_LENGTH)
                # Calculate hip_joint target angle to bring foot to pep_pos [Cheating a lil' here]
                hip_joint_angle_aep = -hip_joint_angle + hip_joint_angle_error
                # Set hip_joint target angle to the angle to bring foot to pep_pos
                sim.simxSetJointTargetPosition(clientID, hip_joint, hip_joint_angle_aep, sim.simx_opmode_streaming)

            ### ATTITUDE CONTROLLER ###
            if (phase == SUPPORT_PHASE):
                imu.update()
                yaw_angle, roll_angle, pitch_angle = transforms3d.euler.mat2euler(imu.rot_mat, axes = "szyx")
                desired_pitch_angle = 0
                pitch_angle_error = desired_pitch_angle - pitch_angle
                pitch_p_gain = 1
                pitch_d_gain = 0
                # Set hip_joint target angle to the angle to bring foot to pep_pos
                pitch_correction = -pitch_p_gain*pitch_angle_error + pitch_d_gain*(pitch_angle - pitch_angle_prev)
                pitch_angle_prev = pitch_angle
                sim.simxSetJointTargetPosition(clientID, hip_joint, pitch_correction, sim.simx_opmode_streaming)
            else:
                pass

        ### CLOSE CONNECTION TO SIM ###
        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    print("=== Programme end ===")