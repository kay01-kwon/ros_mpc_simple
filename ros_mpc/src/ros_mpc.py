#!/usr/bin/env python3.8
import rospy
import os
import threading
from model import double_integrator
from ocp_solver import create_ocp_solver
from acados_template import AcadosOcpSolver
from simple_system_msg.msg import simple_system_state
from simple_system_msg.msg import simple_system_control_input

class ROS_MPC:
    def __init__(self):
        self.ocp = create_ocp_solver()
        self.model = self.ocp.model
        self.acados_ocp_solver = AcadosOcpSolver(self.ocp)





def main():
    rospy.init_node('ROS_MPC')