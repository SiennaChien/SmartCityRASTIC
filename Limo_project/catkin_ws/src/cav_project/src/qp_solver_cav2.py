#!/usr/bin/env python3
import math
import rospy
import numpy as np
from cav_project.msg import limo_info, limo_info_array, QP_solution
from cvxopt import matrix, solvers
from cvxopt.solvers import qp
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from scipy.integrate import odeint

class QPSolverCAV2:
    def __init__(self, ID, otherID):
        self.ID = ID
        self.otherID = otherID
        rospy.init_node("qp_solver_" + self.ID)

        # Publishers and subscribers
        self.qp_solution_pub = rospy.Publisher('/qp_solution_' + self.ID, QP_solution, queue_size=10)
        self.cav_info_sub = rospy.Subscriber('/limo_info_' + self.ID, limo_info, self.cav_info_callback)
        self.other_cav_info_sub = rospy.Subscriber('/limo_info_' + self.otherID, limo_info, self.other_cav_info_callback)
        self.mocap_sub_cav = rospy.Subscriber('/vrpn_client_node/' + self.ID + '/pose', PoseStamped, self.mocap_callback)
        self.mocap_sub_other_cav = rospy.Subscriber('/vrpn_client_node/' + self.otherID + '/pose', PoseStamped, self.mocap_callback_other)

        # Variables to store incoming data
        self.cav_info = None
        self.other_cav_info = None
        self.cav_pose = None
        self.other_cav_pose = None

        # Control parameters
        self.rate = rospy.Rate(10)
        self.u_min = -10  # Minimum control input (deceleration)
        self.u_max = 2    # Maximum control input (acceleration)
        self.v_min = 0.15 # Minimum velocity
        self.v_max = 1.0  # Maximum velocity
        self.deltaSafetyDistance = 300  # Safety distance
        self.phiRearEnd = 1.2
        self.phiLateral = 1.0
        self.L = 0.2  # Wheelbase length in meters (200mm converted to meters)

    def mocap_callback(self, msg):
        # Update pose of CAV2
        self.cav_pose = msg.pose
        self.position_x = msg.pose.position.x * 1000
        self.position_y = msg.pose.position.y * 1000
        self.position_z = msg.pose.position.z * 1000

    def mocap_callback_other(self, msg):
        # Update pose of other CAV
        self.other_cav_pose = msg.pose
        self.other_position_x = msg.pose.position.x * 1000
        self.other_position_y = msg.pose.position.y * 1000
        self.other_position_z = msg.pose.position.z * 1000

    def cav_info_callback(self, msg):
        # Update velocity info of CAV2
        self.cav_info = msg

    def other_cav_info_callback(self, msg):
        # Update velocity info of other CAV
        self.other_cav_info = msg

    def recalc_qp(self):
        infeasible = False
        if not self.cav_info or not self.other_cav_info:
            return

        QP_rows = [[-1, -1, -1, -1]]
        d2s = [-1]

        for other_cav in self.other_cav_info.limo_infos:
            if other_cav.ID.data > 0:
                if other_cav.d2.data >= 0:
                    d2s.append(other_cav.d2.data)
                    row = [
                        other_cav.ID.data,
                        other_cav.origin_dist.data,
                        other_cav.vel.data,
                        other_cav.d1.data
                    ]
                    if other_cav.d1.data + self.deltaSafetyDistance > other_cav.d2.data:
                        infeasible = True
                else:
                    d2s.append(-1)
                    row = [-1, -1, -1, -1]
                QP_rows.append(row)
            else:
                QP_rows[0] = [
                    other_cav.ID.data,
                    other_cav.d1.data,
                    other_cav.vel.data,
                    other_cav.origin_dist.data
                ]
                d2s[0] = other_cav.d2.data

        my_vec = [self.cav_info.origin_dist.data, self.cav_info.vel.data] + d2s
        my_vec = np.array(my_vec)
        qp_mat = np.array(QP_rows)
        u = None

        if len(qp_mat) > 1 and not infeasible:
            u = self.solve_qp(qp_mat, my_vec)
            if u is not None:
                u = u[0]
                print(f"CAV2 QP Solution: {u}")
        if u is None:
            u = self.u_min
        qp_solution = QP_solution()
        qp_solution.u = u
        self.qp_solution_pub.publish(qp_solution)

    def solve_qp(self, qp_mat, my_vec):
        # Ensure all necessary data is received
        if not self.cav_info or not self.other_cav_info or not self.cav_pose or not self.other_cav_pose:
            rospy.loginfo("Waiting for complete data from both vehicles...")
            return

        # Get the relative position and velocity data
        pos_cav = np.array([self.position_x, self.position_y, self.position_z])
        pos_other = np.array([self.other_position_x, self.other_position_y, self.other_position_z])
        vel_cav = self.cav_info.vel.data
        vel_other = self.other_cav_info.vel.data

        # Compute relative distance and velocity
        relative_distance = np.linalg.norm(pos_cav - pos_other) - self.deltaSafetyDistance
        relative_velocity = vel_cav - vel_other

        # OCBF parameters
        ocpar = [-0.593787660013256, 1.41421356237309, 0, 0, 2.38168230431317, 1.68410370801184]
        c = np.array(ocpar)
        x0 = np.array([relative_distance, vel_cav, pos_cav[0], pos_cav[1]])
        eps = 10
        psc = 0.1
        t = rospy.get_time()

        # Reference trajectory
        vd = 0.5 * c[0] * t ** 2 + c[1] * t + c[2]
        u_ref = c[0] * t + c[1]

        # Velocity constraints
        b_vmax = self.v_max - x0[1]
        b_vmin = x0[1] - self.v_min

        # Control Lyapunov Function (CLF)
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        # Constraint matrices
        A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0], [-1, 0]])
        b = np.array([self.u_max, -self.u_min, phi0, b_vmax, b_vmin])

        # Rear-end safety constraints
        if self.other_cav_info:
            xip = pos_other[0]
            vip = vel_other
            h = xip - x0[2] - self.phiRearEnd * x0[1] - self.deltaSafetyDistance
            uminValue = abs(self.u_min)
            hf = h - 0.5 * (vip - x0[1]) ** 2 / uminValue

            if x0[1] <= vip or hf < 0:
                LgB = 1
                LfB = 2 * (vip - x0[1]) + h
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB])
            else:
                LgB = self.phiRearEnd - (vip - x0[1]) / uminValue
                LfB = vip - x0[1]
                if LgB != 0:
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + hf])

            # Lateral safety constraints
            d1 = pos_other[1]
            d2 = pos_cav[1]
            L = self.L

            v0 = vel_other
            bigPhi = self.phiLateral * x0[0] / L
            h = 0.1 * (d2 - d1 - bigPhi * x0[1] - self.deltaSafetyDistance)
            hf = d2 - d1 - 0.5 * (v0 - x0[1]) ** 2 / uminValue - self.phiLateral * v0 * (
                x0[0] + 0.5 * (x0[1] ** 2 - v0 ** 2) / uminValue) / L

            LgB = bigPhi
            LfB = v0 - x0[1] - self.phiLateral * x0[1] ** 2 / L
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])

        # QP problem formulation
        H = matrix([[1, 0], [0, psc]])
        f = matrix([[-u_ref], [0]]).trans()
        H = matrix(H, tc='d')
        f = matrix(f, tc='d')
        A = matrix(A, tc='d')
        b = matrix(b, tc='d')

        # Solve QP problem
        solvers.options['show_progress'] = False
        Solution = solvers.qp(H, f, A, b, options={
            'abstol': 0.0001,
            'reltol': 0.0001,
            'feastol': 0.0001
        })

        # Publish solution
        if Solution['status'] == 'optimal':
            u = Solution['x'].trans()
            qp_solution = QP_solution()
            qp_solution.u = u[0]
            self.qp_solution_pub.publish(qp_solution)
        else:
            rospy.loginfo("No feasible QP solution found. Implementing safety measures...")
            qp_solution = QP_solution()
            qp_solution.u = self.u_min
            self.qp_solution_pub.publish(qp_solution)

    def run(self):
        while not rospy.is_shutdown():
            self.recalc_qp()
            self.rate.sleep()

if __name__ == '__main__':
    solver = QPSolverCAV2("limo155", "limo770")
    solver.run()
