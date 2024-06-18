#!/usr/bin/env python3
import math
import rospy
import numpy as np
from std_msgs.msg import Float64, Bool, Float64MultiArray, String
from cav_project.msg import limo_info, limo_info_array, QP_solution
from cvxopt import matrix, solvers
from cvxopt.solvers import qp
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint


class OtherCAV:
    def __init__(self, otherID, merging_pt_x, merging_pt_y):
        self.otherID = otherID
        self.merging_pt_x = merging_pt_x
        self.merging_pt_y = merging_pt_y
        self.pose = None
        self.velocity = 0
        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.d1 = 0
        self.mocap_sub_other_cav = rospy.Subscriber('/vrpn_client_node/' + self.otherID + '/pose', PoseStamped, self.mocap_callback_other)
        self.other_cav_info_sub = rospy.Subscriber('/limo_info_' + self.otherID, limo_info, self.other_cav_info_callback)

    def mocap_callback_other(self, msg):
        self.pose = msg.pose
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_z = msg.pose.position.z * 1000
        self.d1 = np.sqrt((self.position_x - self.merging_pt_x)**2 + (self.position_y - self.merging_pt_y)**2)

    def other_cav_info_callback(self, msg):
        self.velocity = msg.vel.data

class QPSolverCAV1:
    def __init__(self, ID, otherID, isMain):
        self.ID = ID
        self.otherID = otherID
        self.isMain = isMain

        self.position_x = 0
        self.position_y = 0
        self.position_z = 0
        self.velocity = 0

        self.cav_pose = None
        self.merging_pt_x = 1950
        self.merging_pt_y = -652

        # Reference velocities for CAVs
        self.reference_velocities = {
            'limo770': 0.5,
            'limo155': 0.3,
            'limo654': 0.4,
            'limo876': 0.35,
            'limo975': 0.45
        }

        # Subscribe to the pose and velocity info of the controlled CAV
        self.cav_info_sub = rospy.Subscriber('/limo_info_' + self.ID, limo_info, self.cav_info_callback)
        self.mocap_sub_cav = rospy.Subscriber('/vrpn_client_node/' + self.ID + '/pose', PoseStamped, self.mocap_callback)
        self.qp_solution_pub = rospy.Publisher('/qp_solution_' + self.ID, QP_solution, queue_size=1)

        # Create an OtherCAV instance for the main other CAV
        self.other_cav = OtherCAV(self.otherID, self.merging_pt_x, self.merging_pt_y)
        self.other_cavs = [
            OtherCAV('limo654', self.merging_pt_x, self.merging_pt_y),
            OtherCAV('limo876', self.merging_pt_x, self.merging_pt_y),
            OtherCAV('limo975', self.merging_pt_x, self.merging_pt_y)
        ]

        # Generate the map
        self.generate_map(isMain)

    def generate_map(self, isMain):
        self.right_top_x = 3044
        self.right_top_y = 2536
        self.right_center_x = 2040
        self.right_center_y = 2519
        self.right_bottom_x = 683
        self.right_bottom_y = 2536
        self.left_top_x = 2933
        self.left_top_y = -1980
        self.left_center_x = 1922
        self.left_center_y = -1977
        self.merging_pt_x = 1950
        self.merging_pt_y = -652
        self.lane_width = 450

        self.right_top_activation_range = (self.lane_width / 1.3, self.lane_width * 1.1)
        self.right_center_activation_range = (self.lane_width * 1.1, self.lane_width)
        self.right_bottom_activation_range = (self.lane_width * 1.2, self.lane_width)
        self.left_top_activation_range = (self.lane_width * 1.1, self.lane_width / 1.7)
        self.left_center_activation_range = (self.lane_width / 1.5, self.lane_width)
        self.merging_pt_activation_range = (self.lane_width, self.lane_width / 2)

        self.main_path = self.generate_line(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)
        self.merging_path = self.generate_line(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)

        self.merge_path_PID = (-0.0005, -0.00005, -0.001)
        self.main_path_PID = (-0.0005, -0.00005, -0.001)

        if isMain:
            self.points = [(self.right_center_x, self.right_center_y), (self.left_center_x, self.left_center_y)]
            self.lines = [self.main_path]
            self.dists = [self.calc_distance(self.right_center_x, self.right_center_y, self.left_center_x, self.left_center_y)]
            self.PIDs = [self.main_path_PID]
        else:
            self.points = [(self.right_bottom_x, self.right_bottom_y), (self.merging_pt_x, self.merging_pt_y)]
            self.lines = [self.merging_path]
            self.dists = [self.calc_distance(self.right_bottom_x, self.right_bottom_y, self.merging_pt_x, self.merging_pt_y)]
            self.PIDs = [self.merge_path_PID]

    def generate_line(self, x_1, y_1, x_2, y_2):
        # Generate the line equation Ax + By + C = 0
        A = -(y_2 - y_1)
        B = -(x_1 - x_2)
        C = -(y_1 * (x_2 - x_1) - (y_2 - y_1) * x_1)
        return A, B, C

    def calc_distance(self, x_1, y_1, x_2, y_2):
        # Calculate the Euclidean distance between two points
        return np.sqrt((x_2 - x_1)**2 + (y_2 - y_1)**2)

    def mocap_callback(self, msg):
        # Update pose of CAV
        self.cav_pose = msg.pose
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_z = msg.pose.position.z * 1000


    def cav_info_callback(self, msg):
        # Update velocity of CAV
        self.velocity = msg.vel.data

    def construct_matrix_const(self):
        # Construct the matrix containing information about other CAVs
        matrix_const = []
        for other_cav in [self.other_cav] + self.other_cavs:
            if other_cav.velocity != 0 and other_cav.position_x != 0:
                matrix_const.append([
                    other_cav.otherID,
                    other_cav.position_x,
                    other_cav.velocity,
                    other_cav.d1
                ])
            else:
                matrix_const.append([-1, -1, -1, -1])
        return np.array(matrix_const)

    def solve_qp(self):
        # Main function to solve the QP problem
        pos_cav = np.array([self.position_x, self.position_y, self.position_z])
        vel_cav = self.velocity

        # Constants and parameters
        u_min = -10
        u_max = 2
        phiRearEnd = 0.18
        phiLateral = 0.18
        deltaSafetyDistance = 300
        v_min = 0.15
        v_max = 1

        eps = 10
        psc = 0.1
        t = rospy.get_time()
        ocpar = [-0.593787660013256, 1.41421356237309, 0, 0, 2.38168230431317, 1.68410370801184]
        c = np.array(ocpar)
        vd = self.reference_velocities[self.ID]  # Use the reference velocity for this CAV
        u_ref = c[0] * t + c[1]

        x0 = np.array([pos_cav[0], vel_cav])
        b_vmax = v_max - x0[1]
        b_vmin = x0[1] - v_min

        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        matrix_const = self.construct_matrix_const()

        def solveQP():
            # Function to set up and solve the QP problem
            A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0], [-1, 0]])
            b = np.array([u_max, -u_min, phi0, b_vmax, b_vmin])

            # Rear-end Safety Constraints
            if matrix_const[0][0] != -1:
                xip = matrix_const[0][1]
                h = xip - x0[0] - phiRearEnd * x0[1] - deltaSafetyDistance
                vip = matrix_const[0][2]
                uminValue = abs(u_min)
                hf = h - 0.5 * (vip - x0[1]) ** 2 / uminValue

                if x0[1] <= vip or hf < 0:
                    p = 1
                    LgB = 1
                    LfB = 2 * p * (vip - x0[1]) + p ** 2 * h
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB])
                else:
                    LgB = phiRearEnd - (vip - x0[1]) / uminValue
                    LfB = vip - x0[1]
                    if LgB != 0:
                        A = np.append(A, [[LgB, 0]], axis=0)
                        b = np.append(b, [LfB + hf])

            # Lateral Safety Constraints
            for row_index, row in enumerate(matrix_const):
                if row[0] == -1:
                    continue
                d1 = np.sqrt((row[1] - self.merging_pt_x)**2 + (row[2] - self.merging_pt_y)**2)  # Correct d1 calculation
                d2 = np.sqrt((self.position_x - self.merging_pt_x)**2 + (self.position_y - self.merging_pt_y)**2)-d1  # Correct d2 calculation
                d1=-1
                L = 3500
                v0 = row[2]

                bigPhi = phiLateral * x0[0] / L
                h = 0.1 * (d2 - d1 - bigPhi * x0[1] - deltaSafetyDistance)

                uminValue = abs(u_min)
                hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - phiLateral * v0 * (
                        x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L

                LgB = bigPhi
                LfB = v0 - x0[1] - phiLateral * x0[1] ** 2 / L
                if LgB != 0:
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + h])

            # Define QP problem matrices
            H = matrix([[1, 0], [0, psc]])
            f = matrix([[-u_ref], [0]]).trans()
            H = matrix(H, tc='d')
            f = matrix(f, tc='d')
            A = matrix(A, tc='d')
            b = matrix(b, tc='d')

            # Solve the QP problem
            Solution = solvers.qp(H, f, A, b, options={
                'show_progress': False,
                'abstol': 0.0001,
                'reltol': 0.0001,
                'feastol': 0.0001
            })

            return float(Solution['x'][0]), A, b  # Ensure that only the float value is returned

        u, a, b = solveQP()

        if u is not None:
            #print(f"QP Solution: u = {u}")
            return u
        else:
            #print("QP Solution not found.")
            return None

    def recalc_qp(self):
        # Recalculate the QP solution and publish the control input
        u = self.solve_qp()
        if u is None:
            u = -10  # Ensure u is a float even if not found

        qp_solution_msg = QP_solution()
        qp_solution_msg.u = float(u)  # Ensure u is a float
        self.qp_solution_pub.publish(qp_solution_msg)
        #print(f"Published QP Solution: u = {u}")

    def publish_control(self):
        rospy.init_node('qp_solver_cav1', anonymous=True)
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.recalc_qp()
            rate.sleep()

if __name__ == '__main__':
    solver = QPSolverCAV1('limo770', 'limo155', False)
    solver.publish_control()
