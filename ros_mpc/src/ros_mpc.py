#!/usr/bin/env python3.8
import numpy as np
import rospy
import os
import threading
from model import double_integrator
from ocp_solver import create_ocp_solver
from acados_template import AcadosOcpSolver
from simple_system_msg.msg import simple_system_state
from simple_system_msg.msg import simple_system_control_input
from simple_system_msg.msg import simple_system_ref
class ROS_MPC:
    def __init__(self):

        # Create OCP solver
        self.ocp = create_ocp_solver()
        self.model = self.ocp.model
        self.acados_ocp_solver = AcadosOcpSolver(self.ocp)

        self.y_ref = np.zeros((6,))
        self.y_ref_N = np.zeros((4,))
        self.state = np.zeros((4,))

        # Publisher and subscriber setup
        self.ros_setup()

        # Thread for mpc
        self.mpc_thread = threading.Thread()



    def ros_setup(self):
        self.ref_sub = rospy.Subscriber('/ref',
                                        simple_system_ref,
                                        queue_size = 1)
        self.state_sub = rospy.Subscriber('/state',
                                          simple_system_state,
                                          queue_size = 1)
        self.input_pub = rospy.Publisher('/input',
                                         queue_size = 1)

    def ref_callback(self,msg):
        for i in range(2):
            self.y_ref[i] = msg.ref[i]
            self.y_ref_N[i] = msg.ref[i]

    def state_callback(self,msg):
        for i in range(4):
            self.state[i] = msg.s[i]



def main():
    rospy.init_node('ROS_MPC')