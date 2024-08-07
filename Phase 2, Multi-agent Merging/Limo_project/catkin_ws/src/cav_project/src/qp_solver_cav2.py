#!/usr/bin/env python3
import rospy
import numpy as np
from cvxopt import matrix, solvers
from cav_project.msg import limo_state, limo_state_matrix, QP_solution

class QPSolverCAV2:
    def __init__(self, cav2_id, cav1_id, cav3_id):
        self.cav2_id = cav2_id  # CAV2 on the main road
        self.cav1_id = cav1_id  # CAV1 on the merging road
        self.cav3_id = cav3_id  # CAV3 preceding CAV1 on merging road

        self.u_min = -10  # Minimum control input (deceleration)
        self.u_max = 1 # Maximum control input (acceleration)
        self.phiRearEnd = 4.8 # Reaction time for rear-end safety constraint
        self.phiLateral = 3.3 # Reaction time for lateral safety constraint
        self.deltaSafetyDistance = 0.3# Minimum safety distance (meters)
        self.v_min = 0  # Minimum velocity
        self.v_max = 1  # Maximum velocity

        rospy.init_node("qp_solver_" + self.cav2_id)
        self.qp_solution_pub = rospy.Publisher('/qp_solution_' + self.cav2_id, QP_solution, queue_size=10)
        rospy.Subscriber('/limo_state_matrix', limo_state_matrix, self.limo_state_callback)

        self.state = None
        self.rate = rospy.Rate(10)  # 10 Hz

    def limo_state_callback(self, data):
        for limo in data.limos:
            if limo.limoID == self.cav2_id:
                self.x0 = [limo.d0/1000, limo.vel, limo.d2/1000]
                self.state = [limo.limoID, limo.vel, limo.d0/1000, limo.d1/1000, limo.v1, limo.d2/1000, limo.v2, limo.vd]


    def OCBF_SecondOrderDynamics(self):
        #ocpar = [-0.593787660013256, 1.41421356237309, 0, 0, 2.38168230431317, 1.68410370801184]
        #c = np.array(ocpar)
        x0 = self.x0  # x0[0] is d0, x0[1] is vel, x0[2] is d2
        eps = 10
        psc = 0.1
        #t = 0.1

        # Reference control input
        u_ref = 1 #c[0] * t + c[1]

        # Physical limitations on velocity
        b_vmax = self.v_max - x0[1]
        b_vmin = x0[1] - self.v_min

        # CLF
        vd = self.state[7]
        phi0 = -eps * (x0[1] - vd) ** 2
        phi1 = 2 * (x0[1] - vd)

        # Initial A and b matrices
        A = np.array([[1, 0], [-1, 0], [phi1, -1], [1, 0], [-1, 0]])
        b = np.array([self.u_max, -self.u_min, phi0, b_vmax, b_vmin])

        #print CLF values
        #print(f"CLF phi0: {phi0}, phi1: {phi1}")

        # Rear-end Safety Constraints
        if self.state[3] != -0.001:
            print("qp two, rear end")
            d1 = self.state[3]
            h = d1 - self.phiRearEnd * x0[1] - self.deltaSafetyDistance
            vip = self.state[4]  # Velocity of the preceding vehicle CAV3
            LgB = self.phiRearEnd
            LfB = vip - x0[1]
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])
                rear_end_h = h

                #print rear-end CBF values
                #print(f"Rear-end h: {h}")

        # Lateral Safety Constraint
        if self.state[5] != -0.001:
            L = 3.5  # Length of the merging lane
            d2 = self.state[5]  # Distance d2 from limo_state message
            print("qp two, lateral")
            v0 = self.state[6]  # Velocity of the conflicting vehicle (CAV1)
            bigPhi = self.phiLateral * x0[0] / L
            h = d2 - bigPhi * x0[1] - self.deltaSafetyDistance
            LgB = bigPhi
            LfB = v0 - x0[1] - self.phiLateral * x0[1] ** 2 / L
            if LgB != 0:
                A = np.append(A, [[LgB, 0]], axis=0)
                b = np.append(b, [LfB + h])
                lateral_h = h

                #print lateral CBF values
                #print(f"Lateral h: {h}")
                #current_time = rospy.Time.now() - self.start_time
                #self.lateral_h_values.append(h)
                #self.time_values.append(current_time.to_sec())

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
            #print(f"QP solver error: {e}")
            u = self.u_min

        # Evaluate and ##print the actual constraint values
        #delta = 1
        #print(f"Evaluated CLF constraint: {phi1 * u - phi0 - delta}")
        #if rear_end_h is not None:
            #print(f"Evaluated Rear-end constraint: {-LgB * u + LfB + rear_end_h}")
        #if lateral_h is not None:
            #print(f"Evaluated Lateral constraint: {-LgB * u + LfB + lateral_h}")

        return u

    def recalc_qp(self):
        if self.state is not None:
            u = self.OCBF_SecondOrderDynamics()
            qp_solution_msg = QP_solution()
            qp_solution_msg.u = u
            #print("qp 2 u", u)
            self.qp_solution_pub.publish(qp_solution_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.recalc_qp()
            self.rate.sleep()

if __name__ == '__main__':
    solver = QPSolverCAV2("limo155", "limo770", "limo795")
    solver.run()
