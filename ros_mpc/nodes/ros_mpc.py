#!/usr/bin/env python3.8
import numpy as np
import rospy
from ocp_solver import create_ocp_solver
from acados_template import AcadosOcpSolver
from simple_system_msg.msg import simple_system_state
from simple_system_msg.msg import simple_system_control_input
from simple_system_msg.msg import simple_system_ref
class ROS_MPC:
    def __init__(self):

        # Create OCP solver
        self.ocp = create_ocp_solver.create_ocp_solver()
        self.model = self.ocp.model
        self.acados_ocp_solver = AcadosOcpSolver(self.ocp)
        self.N_horizon = self.acados_ocp_solver.N

        self.y_ref = np.zeros((6,))
        self.y_ref_N = np.zeros((4,))
        self.state = np.zeros((4,))

        self.control_msg = simple_system_control_input()


        # Publisher and subscriber setup
        self.ros_setup()

        # Thread for mpc
        # self.mpc_thread = threading.Thread()



    def ros_setup(self):
        self.ref_sub = rospy.Subscriber('/ref',
                                        simple_system_ref,
                                        self.ref_callback,
                                        queue_size = 1)
        self.state_sub = rospy.Subscriber('/state',
                                          simple_system_state,
                                          self.state_callback,
                                          queue_size = 1)
        self.input_pub = rospy.Publisher('/input',
                                         simple_system_control_input,
                                         queue_size = 1)

    def ref_callback(self,msg):
        for i in range(2):
            self.y_ref[i] = msg.ref[i]
            self.y_ref_N[i] = msg.ref[i]

    def state_callback(self,msg):
        self.state[:] = msg.s[:]
        print(self.state)

        try:
            for i in range(self.N_horizon):
                self.acados_ocp_solver.set(i, "y_ref", self.y_ref)
            self.acados_ocp_solver.set(self.N_horizon, "y_ref", self.y_ref_N)
            U = self.acados_ocp_solver.solve_for_x0(self.state)
            for i in range(2):
                self.control_msg.u[i] = U[i]
            self.input_pub.publish(self.control_msg)
        except rospy.ROSInterruptException:
            rospy.logerr("MPC is not yet ready")

def main():
    rospy.init_node('ROS_MPC', anonymous=True)
    ros_mpc = ROS_MPC()
    ros_rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        ros_rate.sleep()

if __name__ == '__main__':
    main()