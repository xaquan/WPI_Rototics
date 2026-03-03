
import numpy as np
from .ConverterHelper import ConverterHelper

class KinematicModel():
    @staticmethod
    def wrap_to_pi(a):
        """Wrap angle(s) to [-pi, pi)."""
        a = np.asarray(a, dtype=float)
        return (a + np.pi) % (2*np.pi) - np.pi

    def inverse_kinematics_ABB_1400(self, pose):
        """
            Calculate the inverse kinematics solution for a 6-DOF robot manipulator.
            This function computes the joint angles (theta1 through theta6) required to achieve
            a desired end-effector position and orientation. It uses geometric approach to solve
            the inverse kinematics problem for a robot with the specified robot parameters.
            Args:
                pose (geometry_msgs.msg.Pose): The desired end-effector pose as a 3D position and orientation.
            Returns:
                ndarray: A 1D array of 6 float values representing the joint angles [theta1, theta2,
                        theta3, theta4, theta5, theta6] in radians. Each angle corresponds to a
                        joint of the robot manipulator.
            Notes:
                - The function uses robot-specific parameters: d1=475, a1=150, a2=600, a4=120,
                d5=720, d6=85 (all in mm).
                - Multiple kinematic solutions are computed for theta1, theta2, theta3, and theta5.
                The solution with the smallest absolute wrapped angle to π is selected.
            """
        # Robot dimensions
        d1 = 475
        a1 = 150
        a2 = 600
        a4 = 120
        d5 = 720
        d6 = 85

        # Convert quaternion to rotation matrix
        rotation = ConverterHelper.quat_to_rotation_array(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) 

        r11 = rotation[0][0]
        r12 = rotation[0][1]
        r13 = rotation[0][2]
        r21 = rotation[1][0]
        r22 = rotation[1][1]
        r23 = rotation[1][2]
        r31 = rotation[2][0]
        r32 = rotation[2][1] 
        r33 = rotation[2][2]

        # Calculate wrist center position        
        px = pose.position.x - d6 * r13
        py = pose.position.y - d6 * r23
        pz = pose.position.z - d6 * r33

        # Calculate intermediate values for theta1, theta2, theta3
        s = pz - d1
        r0 = np.sqrt(px**2 + py**2)
        r2 = r0 - a1
        r5 = np.sqrt(a4**2 + d5**2)
        r6 = np.sqrt(r2**2 + s**2)

        # Initialize theta array
        theta = np.zeros(6, dtype=float)

        # ---------------------------
        # Theta1
        D = px / r0                      # cos(theta1)
        C = np.sqrt(1.0 - D**2)          # sin(theta1) (positive branch)

        # Two possible solutions for theta1
        theta1_1 = np.arctan2(C, D)
        theta1_2 = np.arctan2(-C, D)
        theta1_candidates = np.array([theta1_1, theta1_2])

        # Select the solution for theta1 that is closest to zero (or any reference angle)
        k = np.argmin(np.abs(self.wrap_to_pi(theta1_candidates)))
        theta[0] = theta1_candidates[k]

        # ---------------------------
        # Theta2
        G = (a2**2 + r6**2 - r5**2) / (2.0 * a2 * r6)   # cos(alpha)
        H = np.sqrt(1.0 - G**2)                         # sin(alpha)

        # Two possible solutions for alpha
        alpha1 = np.arctan2(H, G)
        alpha2 = np.arctan2(-H, G)

        I = s / r6                                      # sin(beta)
        J = np.sqrt(1.0 - I**2)                         # cos(beta)

        # Two possible solutions for beta
        beta1 = np.arctan2(J, I)
        beta2 = np.arctan2(-J, I)

        # Combine alpha and beta to get candidates for theta2
        theta2_candidates = np.array([alpha1 - beta1, alpha1 - beta2, alpha2 - beta1, alpha2 - beta2])

        # Select the solution for theta2 that is closest to zero
        k = np.argmin(np.abs(self.wrap_to_pi(theta2_candidates)))
        theta[1] = theta2_candidates[k]

        # ---------------------------
        # Theta3
        gamma = np.arctan(a4 / d5)

        L = -(a2**2 + r5**2 - r6**2) / (2.0 * a2 * r5)  # cos(sigma)
        M = np.sqrt(1.0 - L**2)                          # sin(sigma)

        # Two possible solutions for sigma
        sigma1 = np.arctan2(M, L)
        sigma2 = np.arctan2(-M, L)

        # Combine gamma and sigma to get candidates for theta3
        theta3_1 = np.pi/2.0 - gamma + sigma1
        theta3_2 = np.pi/2.0 - gamma + sigma2

        # Two candidates for theta3
        theta3_candidates = np.array([theta3_1, theta3_2])
        
        # Select the solution for theta3 that is closest to zero
        k = np.argmin(np.abs(self.wrap_to_pi(theta3_candidates)))
        theta[2] = theta3_candidates[k]

        # ---------------------------
        # Theta4 Theta5 Theta6

        # Precompute sines and cosines for theta1, theta2+theta3
        s1 = np.sin(theta[0])
        c1 = np.cos(theta[0])
        c23 = np.cos(theta[1] + theta[2])
        s23 = np.sin(theta[1] + theta[2])

        # Solve theta4
        D4 = c1*c23*r13 + s1*c23*r23 + s23*r33      # c4s5 
        C4 = s1*r13 - c1*r23                        # s4s5
        theta4 = np.arctan2(D4, C4)              
        theta[3] = theta4   

        # Solve theta5
        D5 = c1*s23*r13 + s1*s23*r23 - c23*r33  
        C5 = np.sqrt(1.0 - D5**2)     

        # Two candidates for theta5
        theta5_1 = np.arctan2(D5, C5)
        theta5_2 = np.arctan2(D5, -C5)    
        theta5_candidates = [theta5_1, theta5_2]

        # Select the solution for theta5 that is closest to zero
        k = np.argmin(np.abs(self.wrap_to_pi(theta5_candidates)))
        theta[4] = theta5_candidates[k]

        # Solve theta6
        D6 = -(c1*s23*r11 + s1*s23*r21 - c23*r31)  # -s5c6 
        C6 =  (c1*s23*r12 + s1*s23*r22 - c23*r32)  #  s5s6
        theta[5] = np.arctan2(D6, C6)

        return theta


# kmodel = KinematicModel()

# kmodel.inverse_kinematics_ABB_1400(
#     position=(100, 200, 300),
#     orientation=(0.707, 0, 0.707, 0))

