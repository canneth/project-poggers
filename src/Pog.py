
import sim
import numpy as np
import transforms3d
import time

from InertialMeasurementUnit import InertialMeasurementUnit as Imu
from LinearEncoder import LinearEncoder
from ForceSensor import ForceSensor
from RotaryEncoder import RotaryEncoder

class Pog:
    def __init__(self, clientID_arg, world_frame, body, hip_joint_x, hip, hip_joint_y, leg_cylinder, leg_joint, leg_piston, foot_force_sensor, foot):
        self.clientID = clientID_arg
        self.world_frame = world_frame
        self.body = body
        self.hip_joint_x = hip_joint_x
        self.hip = hip
        self.hip_joint_y = hip_joint_y
        self.leg_cylinder = leg_cylinder
        self.leg_joint = leg_joint
        self.leg_piston = leg_piston
        self.foot_force_sensor = foot_force_sensor
        self.foot = foot

        return_code, self.p_body_to_hip = sim.simxGetObjectPosition(self.clientID, self.body, self.hip, sim.simx_opmode_blocking)
        return_code, self.p_hip_to_foot_home = sim.simxGetObjectPosition(self.clientID, self.hip, self.foot, sim.simx_opmode_blocking)

        # self.target_vel = np.zeros(2)
        self.target_vel = np.array([0, 99])

        self.SUPPORT_PHASE = 1
        self.TRANSFER_PHASE = 2

        self.NEUTRAL_LEG_LENGTH = np.linalg.norm(self.p_hip_to_foot_home)

        ### SENSORS ###
        self.imu = Imu(self.clientID, self.body, self.world_frame)
        self.leg_encoder = LinearEncoder(self.clientID, self.leg_cylinder, self.leg_piston)
        self.foot_sensor = ForceSensor(self.clientID, self.foot_force_sensor)
        self.hip_encoder_x = RotaryEncoder(self.clientID, self.hip_joint_x)
        self.hip_encoder_y = RotaryEncoder(self.clientID, self.hip_joint_y)

        self.leg_extension = None
        self.leg_extension_dot = None
        self.leg_extension_dot_new = None
        self.leg_extension_dotdot = -10

        self.hip_x_neutral_angle = 0
        self.hip_y_neutral_angle = 0

        self.phase = self.TRANSFER_PHASE
        self.body_vel_x = 0
        self.body_vel_y = 0

        self.transfer_start_time = 0
        self.transfer_start = True

    def __repr__(self):
        print("An instance of the custom Pog class")

    def simStep(self):
        """
        This function runs a single cycle of the controllers. Run this in a while loop!
        """
        
        self.foot_sensor.update()
        foot_force = np.linalg.norm(self.foot_sensor.force_vec)
        if (foot_force >= 5.0):
            self.phase = self.SUPPORT_PHASE
        else:
            self.phase = self.TRANSFER_PHASE
        
        ### HOP-HEIGHT CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            # Check for nadir of support self.phase by using leg extension as heuristic
            self.leg_encoder.update()
            self.leg_extension_new = self.leg_encoder.distance
            if self.leg_extension == None:
                self.leg_extension = self.leg_extension_new
            if (self.leg_extension_new != self.leg_extension):
                self.leg_extension_dot_new = self.leg_extension_new - self.leg_extension
                if self.leg_extension_dot == None:
                    self.leg_extension_dot = self.leg_extension_dot_new
                if (self.leg_extension_dot_new != self.leg_extension_dot):
                    self.leg_extension_dotdot = self.leg_extension_dot_new - self.leg_extension_dot
                    self.leg_extension_dot = self.leg_extension_dot_new
                self.leg_extension = self.leg_extension_new
            if (self.leg_extension_dotdot >= 0 and self.leg_extension_dot >= 0):
                # At nadir of support self.phase, extend leg
                sim.simxSetJointTargetPosition(self.clientID, self.leg_joint, 0.05, sim.simx_opmode_streaming)
            self.transfer_start = False
        else:
            if not self.transfer_start:
                self.transfer_start_time = time.time()
                self.transfer_start = True
            if self.transfer_start:
                transfer_time_elapsed = time.time() - self.transfer_start_time
                if (transfer_time_elapsed >= 0.1):
                    # Return leg to neutral length
                    sim.simxSetJointTargetPosition(self.clientID, self.leg_joint, 0.00, sim.simx_opmode_streaming)
                else:
                    # Retract leg completely
                    sim.simxSetJointTargetPosition(self.clientID, self.leg_joint, -0.05, sim.simx_opmode_streaming)

        ### VELOCITY CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            self.hip_encoder_x.update()
            self.hip_encoder_y.update()
            self.hip_x_neutral_angle = self.hip_encoder_x.angle
            self.hip_y_neutral_angle = self.hip_encoder_y.angle
        else:
            """
            Using the heuristic cg_print/support_phase_duration to estimate
            self.body_vel causes system to oscillate out of control over time.
            I suspect it's because the errors in self.body_vel grow proportionately as
            the velocity error grows.

            Using the true self.body_vel derived from the simulator works like a charm.
            This lends credibility to the above suspicion.
            """
            # Get body_vel
            return_code, linear_vel, angular_vel = sim.simxGetObjectVelocity(self.clientID, self.body, sim.simx_opmode_streaming)
            self.imu.update()
            vel_vec = np.dot(np.linalg.inv(self.imu.rot_mat), np.reshape(np.asarray(linear_vel), (3, 1)))
            self.body_vel_x = vel_vec[0]
            self.body_vel_y = vel_vec[1]
            desired_vel_x = self.target_vel[0]
            desired_vel_y = self.target_vel[1]
            # Servo for foot_forward_distance
            x_vel_p_gain = 0.0965
            y_vel_p_gain = 0.0965
            foot_forward_x_distance = -x_vel_p_gain*(desired_vel_x - self.body_vel_x)
            foot_forward_y_distance = -y_vel_p_gain*(desired_vel_y - self.body_vel_y)
            print("forward_x: %.4f | forward_y: %.4f" %(foot_forward_x_distance, foot_forward_y_distance))
            # # Constrain foot_forward_distance
            # if (abs(foot_forward_x_distance) >= self.NEUTRAL_LEG_LENGTH*0.99):
            #     foot_forward_x_distance = self.NEUTRAL_LEG_LENGTH*0.99*(foot_forward_x_distance/abs(foot_forward_x_distance))
            # if (abs(foot_forward_y_distance) >= self.NEUTRAL_LEG_LENGTH*0.99):
            #     foot_forward_y_distance = self.NEUTRAL_LEG_LENGTH*0.99*(foot_forward_y_distance/abs(foot_forward_y_distance))
            hip_R_neutral = transforms3d.euler.euler2mat(0, self.hip_y_neutral_angle, self.hip_x_neutral_angle, axes = "szyx")
            p_body_to_foot_neutral = np.reshape(np.reshape(np.asarray(self.p_body_to_hip), (3, 1)) + np.dot(hip_R_neutral, np.reshape(np.asarray(self.p_hip_to_foot_home), (3, 1))), 3)
            b_p_body_to_foot_target = np.reshape(np.reshape(p_body_to_foot_neutral, (3, 1)) + np.reshape(np.asarray([foot_forward_x_distance, foot_forward_y_distance, 0]), (3, 1)), 3)

            if (abs(b_p_body_to_foot_target[0]) >= self.NEUTRAL_LEG_LENGTH*0.99):
                b_p_body_to_foot_target[0] = self.NEUTRAL_LEG_LENGTH*0.99*(b_p_body_to_foot_target[0]/abs(b_p_body_to_foot_target[0]))
            if (abs(b_p_body_to_foot_target[1]) >= self.NEUTRAL_LEG_LENGTH*0.99):
                b_p_body_to_foot_target[1] = self.NEUTRAL_LEG_LENGTH*0.99*(b_p_body_to_foot_target[1]/abs(b_p_body_to_foot_target[1]))
            
            alpha = np.arcsin(b_p_body_to_foot_target[1]/self.NEUTRAL_LEG_LENGTH)
            b_R_hx = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
            hx_R_hy = np.dot(transforms3d.euler.euler2mat(0, 0, alpha, axes = "sxyz"), np.asarray([[0, 1, 0], [0, 0, 1], [1, 0, 0]]))
            hy_R_hx = np.linalg.inv(hx_R_hy)
            hx_R_b = np.linalg.inv(b_R_hx)
            hy_p_hip_to_foot_target = np.dot(np.dot(hy_R_hx, hx_R_b), (b_p_body_to_foot_target - np.asarray(self.p_body_to_hip)))
            beta = np.arcsin(hy_p_hip_to_foot_target[1]/np.linalg.norm(hy_p_hip_to_foot_target))

            hip_x_target_angle = alpha
            hip_y_target_angle = beta

            # print("x_target_angle: %.4f | y_target_angle: %.4f" %(hip_x_target_angle, hip_y_target_angle))
            sim.simxSetJointTargetPosition(self.clientID, self.hip_joint_x, hip_x_target_angle, sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(self.clientID, self.hip_joint_y, hip_y_target_angle, sim.simx_opmode_streaming)

        ### ATTITUDE CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            self.imu.update()
            yaw_angle, roll_angle, pitch_angle = transforms3d.euler.mat2euler(self.imu.rot_mat, axes = "szyx")
            desired_roll_angle = 0
            desired_pitch_angle = 0
            roll_angle_error = desired_roll_angle - roll_angle
            pitch_angle_error = desired_pitch_angle - pitch_angle
            roll_p_gain = 0.92
            pitch_p_gain = 0.92
            roll_correction = -roll_p_gain*roll_angle_error
            pitch_correction = -pitch_p_gain*pitch_angle_error
            sim.simxSetJointTargetPosition(self.clientID, self.hip_joint_x, pitch_correction, sim.simx_opmode_streaming)
            sim.simxSetJointTargetPosition(self.clientID, self.hip_joint_y, roll_correction, sim.simx_opmode_streaming)
        else:
            pass