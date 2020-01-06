
import sim
import numpy as np
import transforms3d
import time

from InertialMeasurementUnit import InertialMeasurementUnit as Imu
from LinearEncoder import LinearEncoder
from ForceSensor import ForceSensor
from RotaryEncoder import RotaryEncoder

class Pog2:
    def __init__(
        self,
        clientID_arg,
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
    ):
        self.clientID = clientID_arg
        self.world_frame = world_frame
        self.body = body
        self.hip_joint_x = hip_joint_x
        self.hip = hip
        self.hip_joint_y = hip_joint_y
        self.leg_actuator_anchor_body_x = leg_actuator_anchor_body_x
        self.leg_actuator_anchor_body_y = leg_actuator_anchor_body_y
        self.leg_actuator_x = leg_actuator_x
        self.leg_actuator_y = leg_actuator_y
        self.leg_actuator_anchor_leg_x = leg_actuator_anchor_leg_x
        self.leg_actuator_anchor_leg_y = leg_actuator_anchor_leg_y
        self.leg_cylinder = leg_cylinder
        self.leg_actuator_extension = leg_actuator_extension
        self.leg_piston = leg_piston
        self.foot_force_sensor = foot_force_sensor
        self.foot = foot
        self.debug_flag = debug_flag

        self.target_vel = np.zeros(2)

        ### SENSORS ###
        self.imu = Imu(self.clientID, self.body, self.world_frame)
        self.leg_encoder = LinearEncoder(self.clientID, self.leg_cylinder, self.leg_piston)
        self.foot_sensor = ForceSensor(self.clientID, self.foot_force_sensor)
        self.hip_encoder_x = RotaryEncoder(self.clientID, self.hip_joint_x)
        self.hip_encoder_y = RotaryEncoder(self.clientID, self.hip_joint_y)
        self.leg_actuator_x_encoder = LinearEncoder(self.clientID, self.leg_actuator_anchor_body_x, self.leg_actuator_anchor_leg_x)
        self.leg_actuator_y_encoder = LinearEncoder(self.clientID, self.leg_actuator_anchor_body_y, self.leg_actuator_anchor_leg_y)

        ### CONSTANTS ###
        self.SUPPORT_PHASE = 1
        self.TRANSFER_PHASE = 2
        return_code, self.h_p_hip_to_foot_home = sim.simxGetObjectPosition(self.clientID, self.foot, self.hip, sim.simx_opmode_blocking)
        self.LEG_LENGTH_NEUTRAL = abs(np.linalg.norm(self.h_p_hip_to_foot_home))
        return_code, self.h_p_hip_to_leg_anchor = sim.simxGetObjectPosition(self.clientID, self.leg_actuator_anchor_leg_x, self.hip, sim.simx_opmode_blocking)
        self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE = abs(np.linalg.norm(np.asarray(self.h_p_hip_to_leg_anchor)))
        return_code, self.b_p_body_to_hip = sim.simxGetObjectPosition(self.clientID, self.hip, self.body, sim.simx_opmode_blocking)
        self.BODY_TO_HIP_DISTANCE = abs(np.linalg.norm(np.asarray(self.b_p_body_to_hip)))
        return_code, self.b_p_body_anchor_to_leg_anchor_x = sim.simxGetObjectPosition(self.clientID, self.leg_actuator_anchor_leg_x, self.leg_actuator_anchor_body_x, sim.simx_opmode_blocking)
        return_code, self.b_p_body_anchor_to_leg_anchor_y = sim.simxGetObjectPosition(self.clientID, self.leg_actuator_anchor_leg_y, self.leg_actuator_anchor_body_y, sim.simx_opmode_blocking)
        self.LEG_ACTUATOR_X_LENGTH_NEUTRAL = abs(np.linalg.norm(np.asarray(self.b_p_body_anchor_to_leg_anchor_x)))
        self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL = abs(np.linalg.norm(np.asarray(self.b_p_body_anchor_to_leg_anchor_y)))
        return_code, self.h_d_x = sim.simxGetObjectPosition(self.clientID, self.leg_actuator_anchor_body_x, self.hip, sim.simx_opmode_blocking)
        return_code, self.h_d_y = sim.simxGetObjectPosition(self.clientID, self.leg_actuator_anchor_body_y, self.hip, sim.simx_opmode_blocking)

        ### VARIABLES ###
        # For tracking current phase
        self.phase = self.TRANSFER_PHASE
        # For hop-height controller
        self.leg_extension = None
        self.leg_extension_new = None
        self.leg_extension_dot = None
        self.leg_extension_dot_new = None
        self.leg_extension_dotdot = -10
        self.transfer_start_time = 0
        self.transfer_start = True
        # For velocity controller
        self.leg_actuator_x_length = self.LEG_ACTUATOR_X_LENGTH_NEUTRAL
        self.leg_actuator_y_length = self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL
        self.body_vel = np.zeros(2)
        

    def __repr__(self):
        print("An instance of the custom Pog class")

    def moveLegActuatorX(self, extension):
        sim.simxSetJointTargetPosition(self.clientID, self.leg_actuator_x, extension, sim.simx_opmode_streaming)
    
    def moveLegActuatorY(self, extension):
        sim.simxSetJointTargetPosition(self.clientID, self.leg_actuator_y, extension, sim.simx_opmode_streaming)
    
    def moveLegActuatorExtension(self, extension):
        sim.simxSetJointTargetPosition(self.clientID, self.leg_actuator_extension, extension, sim.simx_opmode_streaming)
    
    def moveLegToAngles(self, angle_x, angle_y):
        """
        Calculates required leg_actuator_x extension length for the desired leg angles and
        moves the leg accordingly.
        """
        alpha, beta = self.legAnglesToActuatorExtensionLengths(angle_x, angle_y)
        self.moveLegActuatorX(alpha)
        self.moveLegActuatorY(beta)
    
    def ikFoot(self, b_p_body_to_target):
        """
        Calculates the actuator angles for leg_actuator_x and leg_actuator_y to bring the foot
        to the target position b_p_body_to_target.

        ARGUMENTS:
        + b_p_body_to_target: An array-like type of size 3; the 3-vector position of the foot target.

        RETURNS:
        + alpha: leg_actuator_x value that will bring foot to target.
        + beta: leg_actuator_y value that will bring foot to target.
        """
        if (self.debug_flag):
            print("##### self.ikFoot() START #####")
        if (self.debug_flag):
            print("b_p_body_to_target: {}".format(b_p_body_to_target))
            print("b_p_body_to_hip: {}".format(self.b_p_body_to_hip))
        b_p_hip_to_target = np.asarray(b_p_body_to_target) - np.asarray(self.b_p_body_to_hip)
        if (self.debug_flag):
            print("b_p_hip_to_target: {}".format(b_p_hip_to_target))
        h_p_hip_to_target = b_p_hip_to_target
        if (self.debug_flag):
            print("h_p_hip_to_target: {}".format(h_p_hip_to_target))
        h_p_hip_to_leg_anchor_target = (self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE/self.LEG_LENGTH_NEUTRAL)*h_p_hip_to_target
        if (self.debug_flag):
            print("hip-anchor to hip-target ratio: {}".format((self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE/self.LEG_LENGTH_NEUTRAL)))
            print("HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE: {}".format(self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE))
            print("LEG_LENGTH_NEUTRAL: {}".format(self.LEG_LENGTH_NEUTRAL))
            print("h_p_hip_to_leg_anchor_target: {}".format(h_p_hip_to_leg_anchor_target))
        if (self.debug_flag):
            print("h_d_x: {}".format(self.h_d_x))
            print("h_d_y: {}".format(self.h_d_y))
        leg_actuator_x_vec = h_p_hip_to_leg_anchor_target - np.asarray(self.h_d_x)
        leg_actuator_y_vec = h_p_hip_to_leg_anchor_target - np.asarray(self.h_d_y)
        if (self.debug_flag):
            print("leg_actuator_x_vec: {}".format(leg_actuator_x_vec))
            print("leg_actuator_y_vec: {}".format(leg_actuator_y_vec))
            print("leg_actuator_x_length_req: {}".format(np.linalg.norm(leg_actuator_x_vec)))
            print("leg_actuator_y_length_req: {}".format(np.linalg.norm(leg_actuator_y_vec)))
        alpha = np.linalg.norm(leg_actuator_x_vec) - self.LEG_ACTUATOR_X_LENGTH_NEUTRAL
        beta = np.linalg.norm(leg_actuator_y_vec) - self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL
        if (self.debug_flag):
            print("LEG_ACTUATOR_X_LENGTH_NEUTRAL: {}".format(np.linalg.norm(self.LEG_ACTUATOR_X_LENGTH_NEUTRAL)))
            print("LEG_ACTUATOR_Y_LENGTH_NEUTRAL: {}".format(np.linalg.norm(self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL)))
            print("alpha: {}".format(alpha))
            print("beta: {}".format(beta))
        
        if (self.debug_flag):
            print("##### self.ikFoot() END #####\n")

        return alpha, beta
    
    def fkFoot(self, alpha, beta, leg_length):
        """
        Calculates the foot position given the leg_actuator_x and leg_actuator_y extension lengths and leg_length.

        ARGUMENTS:
        + alpha: A float; the extension length of leg_actuator_x.
        + beta: A float; the extension length of leg_actuator_y.
        + leg_length: A float; the length of the leg from hip to foot.

        RETURNS:
        + b_p_body_to_foot: A size 3 array; the 3-vector position of the foot wrt body in the body frame.
        """
        """
        hip_joint_x
        |__ hip
            |__ hip_joint_y
        """
        if (self.debug_flag):
            print("##### self.fkFoot START #####")
        if (self.debug_flag):
            print("alpha: {first} | beta: {second}".format(first = alpha, second = beta))
        a_x = abs(np.linalg.norm(self.h_d_x))
        a_y = abs(np.linalg.norm(self.h_d_y))
        b = self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE
        actuator_length_x = alpha + self.LEG_ACTUATOR_X_LENGTH_NEUTRAL
        actuator_length_y = beta + self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL
        if (self.debug_flag):
            print("actuator_length_x: {first} | actuator_length_y: {second}".format(first = actuator_length_x, second = actuator_length_y))
        numerator_x = (actuator_length_x**2 - a_x**2 - b**2)
        numerator_y = (actuator_length_y**2 - a_y**2 - b**2)
        denominator_x = (-2*a_x*b)
        denominator_y = (-2*a_y*b)
        if abs(abs(numerator_x) - abs(denominator_x)) <= 0.001: # Remove nan from arccos due to margin of error
            numerator_x = (numerator_x/abs(numerator_x))*abs(denominator_x)
        if abs(abs(numerator_y) - abs(denominator_y)) <= 0.001: # Remove nan from arccos due to margin of error
            numerator_y = (numerator_y/abs(numerator_y))*abs(denominator_y)
        pre_arccos_x = numerator_x/denominator_x
        pre_arccos_y = numerator_y/denominator_y
        theta_x = np.arccos(pre_arccos_x)
        theta_y = np.arccos(pre_arccos_y)
        if (self.debug_flag):
            print("numerator_x: {} | numerator_y: {}".format(numerator_x, numerator_y))
            print("denominator_x: {} | denominator_y: {}".format(denominator_x, denominator_y))
            print("pre_arccos_x: {} | pre_arccos_y: {}".format(pre_arccos_x, pre_arccos_y))
            print("theta_x: {first} | theta_y: {second}".format(first = theta_x, second = theta_y))
        hip_angle_y = -(theta_x - np.pi/2)
        hip_angle_x = theta_y - np.pi/2
        if (self.debug_flag):
            print("hip_angle_x: {first} | hip_angle_y: {second}".format(first = hip_angle_x, second = hip_angle_y))
        R_x = transforms3d.euler.euler2mat(hip_angle_x, 0, 0, axes = "sxyz")
        R_y = transforms3d.euler.euler2mat(0, hip_angle_y, 0, axes = "sxyz")
        if (self.debug_flag):
            print("R_x: {}".format(R_x))
            print("R_y: {}".format(R_y))
        p_anchor_0 = np.asarray([0, 0, -self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE])
        p_anchor = np.reshape(np.dot(R_y, np.dot(R_x, np.reshape(p_anchor_0, (3, 1)))), 3)
        if (self.debug_flag):
            print("p_anchor: {}".format(p_anchor))
        h_p_hip_to_foot = (p_anchor/(np.linalg.norm(p_anchor)))*(leg_length)
        if (self.debug_flag):
            print("h_p_hip_to_foot: {}".format(h_p_hip_to_foot))
        b_p_body_to_foot = np.asarray(self.b_p_body_to_hip) + h_p_hip_to_foot
        if (self.debug_flag):
            print("b_p_body_to_foot: {}".format(b_p_body_to_foot))


        if (self.debug_flag):
            print("##### self.fkFoot END #####")
        return b_p_body_to_foot

    def legAnglesToActuatorExtensionLengths(self, leg_x_angle, leg_y_angle):
        """
        Calculates the extension lengths of leg_actuator_x and leg_actuator_y for the
        desired leg_x_angle and leg_y_angle.

        ARGUMENTS:
        + leg_x_angle: A float; the angle as recorded by hip_encoder_x.
        + leg_y_angle: A float; the angle as recorded by hip_encoder_y.

        RETURNS:
        + alpha: A float; the extension length of leg_actuator_x.
        + beta: A float; the extension length of leg_actuator_y.
        """
        if (self.debug_flag):
            print("##### self.legAnglesToActuatorExtensionLengths() START #####")
        
        if self.debug_flag:
            print("leg_x_angle: {first} | leg_y_angle: {second}".format(first = leg_x_angle, second = leg_y_angle))
        if self.debug_flag:
            print("leg_x_angle (corrected): {first} | leg_y_angle (corrected): {second}".format(first = leg_x_angle, second = leg_y_angle))
        a_x = abs(np.linalg.norm(self.h_d_x))
        a_y = abs(np.linalg.norm(self.h_d_y))
        if self.debug_flag:
            print("a_x: {first} | a_y: {second}".format(first = a_x, second = a_y))
        b = self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE
        if self.debug_flag:
            print("HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE: {}".format(self.HIP_TO_LEG_ACTUATOR_ANCHOR_DISTANCE))
        leg_actuator_x_length_required = np.sqrt(a_x**2 + b**2 - 2*a_x*b*np.cos(leg_y_angle + np.pi/2))
        leg_actuator_y_length_required = np.sqrt(a_y**2 + b**2 - 2*a_y*b*np.cos(leg_x_angle + np.pi/2))
        if self.debug_flag:
            print("leg_actuator_x_length_required: {first} | leg_actuator_y_length_required: {second}".format(first = leg_actuator_x_length_required, second = leg_actuator_y_length_required))
        alpha = leg_actuator_x_length_required - self.LEG_ACTUATOR_X_LENGTH_NEUTRAL
        beta = leg_actuator_y_length_required - self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL
        if self.debug_flag:
            print("alpha: {first} | beta: {second}".format(first = alpha, second = beta))

        if (self.debug_flag):
            print("##### self.legAnglesToActuatorExtensionLengths() END #####")
        return alpha, beta

    def moveFootToTarget(self, b_p_body_to_target):
        alpha, beta = self.ikFoot(b_p_body_to_target)
        self.moveLegActuatorX(alpha)
        self.moveLegActuatorY(beta)

    def simStep(self):
        self.foot_sensor.update()
        foot_force = np.linalg.norm(self.foot_sensor.force_vec)
        if (foot_force >= 5.0):
            self.phase = self.SUPPORT_PHASE
        else:
            self.phase = self.TRANSFER_PHASE
        
        ### HOP-HEIGHT CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            self.moveLegActuatorExtension(0.02)
            self.transfer_start = False
        else:
            if not self.transfer_start:
                self.transfer_start_time = time.time()
                self.transfer_start = True
            if self.transfer_start:
                transfer_time_elapsed = time.time() - self.transfer_start_time
                if (transfer_time_elapsed >= 0.1):
                    # Return leg to neutral length
                    self.moveLegActuatorExtension(0)
                else:
                    # Retract leg completely
                    self.moveLegActuatorExtension(-0.05)

        ### ATTITUDE CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            self.imu.update()
            yaw_angle, roll_angle, pitch_angle = transforms3d.euler.mat2euler(self.imu.rot_mat, axes = "szyx")
            desired_roll_angle = 0
            desired_pitch_angle = 0
            roll_angle_error = desired_roll_angle - roll_angle
            pitch_angle_error = desired_pitch_angle - pitch_angle
            roll_p_gain = 0
            pitch_p_gain = 1
            roll_correction = -roll_p_gain*roll_angle_error
            pitch_correction = -pitch_p_gain*pitch_angle_error
            # Convert angles into actuator lengths
            self.moveLegToAngles(pitch_correction, roll_correction)
        else:
            # Attitude can only be controlled when foot is in contact with the ground
            pass

        ### VELOCITY CONTROLLER ###
        if (self.phase == self.SUPPORT_PHASE):
            self.leg_actuator_x_encoder.update()
            self.leg_actuator_y_encoder.update()
            self.leg_actuator_x_length = self.leg_actuator_x_encoder.distance
            self.leg_actuator_y_length = self.leg_actuator_y_encoder.distance
            # Get body_vel
            return_code, linear_vel, angular_vel = sim.simxGetObjectVelocity(self.clientID, self.body, sim.simx_opmode_streaming)
            self.imu.update()
            vel_vec = np.reshape(np.dot(np.linalg.inv(self.imu.rot_mat), np.reshape(np.asarray(linear_vel), (3, 1))), 3)
            self.body_vel = vel_vec[:2]
        else:
            """
            Using the heuristic cg_print/support_phase_duration to estimate
            self.body_vel causes system to oscillate out of control over time.
            I suspect it's because the errors in self.body_vel grow proportionately as
            the velocity error grows.

            Using the true self.body_vel derived from the simulator works like a charm.
            This lends credibility to the above suspicion.
            """
            target_vel = self.target_vel # For notational convenience
            # Servo for foot_offset
            p_gains = np.asarray([0, 0.07])
            foot_target_offset = np.multiply(-p_gains, (target_vel - self.body_vel))
            print("foot_target_offset: {}".format(foot_target_offset))
            # The last reading of leg_actuator_x_length is taken right at the end of the support phase
            no_accel_x_extension = -(self.leg_actuator_x_length - self.LEG_ACTUATOR_X_LENGTH_NEUTRAL)
            no_accel_y_extension = -(self.leg_actuator_y_length - self.LEG_ACTUATOR_Y_LENGTH_NEUTRAL)
            neutral_foot_target = self.fkFoot(no_accel_x_extension, no_accel_y_extension, self.LEG_LENGTH_NEUTRAL)
            foot_target = neutral_foot_target + np.append(foot_target_offset, 0)
            self.moveFootToTarget(foot_target)
            
        
