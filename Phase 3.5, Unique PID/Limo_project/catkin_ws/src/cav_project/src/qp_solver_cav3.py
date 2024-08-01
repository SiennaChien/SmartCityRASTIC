#!/usr/bin/env python3
import rospy
import numpy as np
from cvxopt import matrix, solvers
from cav_project.msg import limo_state, limo_state_matrix, QP_solution

class QPSolverCAV3:
    def __init__(self, cav3_id, cav2_id, cav1_id, cav4_id,cav5_id):
        self.cav3_id = cav3_id  # CAV3 with the solver (first lateral)
        self.cav2_id = cav2_id  # CAV2 preceding CAV1 (rear-end constraint)s
        self.cav1_id = cav1_id  # receeding
        self.cav4_id = cav4_id  # CAVs4 for second lateral constraint
        self.cav5_id = cav5_id
        self.u_min = -10  # Minimum control input (deceleration)
        self.u_max = 1  # Maximum control input (acceleration)
        self.phiRearEnd = 0.1  # Reaction time for rear-end safety constraint
        self.phiLateral = 0.1  # Reaction time for lateral safety constraint
        self.deltaSafetyDistance = 0.7  # Minimum safety distance (meters)
        self.v_min = 0  # Minimum velocity
        self.v_max = 1  # Maximum velocity

        rospy.init_node("qp_solver_" + self.cav3_id)
        self.qp_solution_pub = rospy.Publisher('/qp_solution_' + self.cav3_id, QP_solution, queue_size=10)
        rospy.Subscriber('/limo_state_matrix', limo_state_matrix, self.limo_state_callback)

        self.state = None
        self.rate = rospy.Rate(10)  # 10 Hz

    def limo_state_callback(self, data):
        for limo in data.limos:
            if limo.limoID == self.cav3_id:
                self.x0 = [limo.d0 / 1000, limo.vel, limo.d2 / 1000, limo.d3 / 1000]
                self.state = [limo.limoID, limo.vel, limo.d0 / 1000, limo.d1 / 1000, limo.v1, limo.d2 / 1000, limo.v2, limo.vd, limo.d3 / 1000, limo.v3, limo.l2/1000, limo.l3/1000]

    def OCBF_SecondOrderDynamics(self):
        x0 = self.x0  # x0[0] is d0, x0[1] is vel, x0[2] is d2, x0[3] is d3
        eps = 10
        psc = 0.1
        vd = self.state[7]
        l2= self.state[10]
        l3= self.state[11]


        # Reference control input
        u_ref = 1

        # Physical limitations on velocity
        b_vmax = self.v_max - x0[1]
        b_vmin = x0[1] - self.v_min

        # CLF
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        # Initial A and b matrices
        A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0]])
        b = np.array([self.u_max, -self.u_min, phi0, b_vmax])

        # Rear-end Safety Constraints
        if self.state[3] != -0.001:
            print("Applying rear-end CBF")
            d1 = self.state[3]
            h = d1 - self.phiRearEnd * x0[1] - self.deltaSafetyDistance
            vip = self.state[4]  # Velocity of the preceding vehicle
            LgB = self.phiRearEnd
            LfB = vip - x0[1]
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])
                print((LfB+h)/LgB)

        # Lateral Safety Constraints
        lateral_constraints = [(self.state[5], self.state[6], l2), (self.state[8], self.state[9], l3)]  # (d2, v2), (d3, v3)
        #L = 3.5  # Length of the merging lane

        for (d, v, L) in lateral_constraints:
            if d != -0.001:
                print("Applying lateral CBF")
                bigPhi = self.phiLateral * x0[0] / 2*L
                h = d - bigPhi * x0[1] -0.4
                LgB = bigPhi
                LfB = v - x0[1] - self.phiLateral * x0[1] ** 2 / L
                if LgB != 0:
                    A = np.append(A, [[LgB, 0]], axis=0)
                    b = np.append(b, [LfB + h])

        # QP formulation
        H = matrix([[1, 0], [0, psc]], tc='d')
        f = matrix([[-u_ref], [0]], (2, 1), tc='d')  # Ensure f is a 2x1 column vector
        A = matrix(A, tc='d')
        b = matrix(b, tc='d')

        solvers.options['show_progress'] = False
        try:
            Solution = solvers.qp(H, f, A, b)
            u = Solution['x'][0]
        except ValueError as e:
            print(f"QP solver error: {e}")
            u = self.u_min

        return u

    def recalc_qp(self):
        if self.state is not None:
            u = self.OCBF_SecondOrderDynamics()
            qp_solution_msg = QP_solution()
            qp_solution_msg.u = u
            print("Computed control input u qp 3:", u)
            self.qp_solution_pub.publish(qp_solution_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.recalc_qp()
            self.rate.sleep()

if __name__ == '__main__':
    solver = QPSolverCAV3("limo795", "limo155", "limo770", "limo789", "limo799")
    solver.run()
