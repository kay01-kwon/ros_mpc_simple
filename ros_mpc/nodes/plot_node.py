#! /usr/bin/env python3.8
import rospy
import matplotlib.pyplot as plt
from simple_system_msg.msg import simple_system_state
from simple_system_msg.msg import simple_system_control_input

class PlotNode():
    def __init__(self):
