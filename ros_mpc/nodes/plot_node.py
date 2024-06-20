#! /usr/bin/env python3.8
import numpy as np
import rospy
import message_filters
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from simple_system_msg.msg import simple_system_state, simple_system_control_input

class PlotNode():
    def __init__(self):
        # State and control input to plot
        self.state_msg = np.zeros((4,))
        self.input_msg = np.zeros((2,))

        self.x_data = []
        self.y_data = []

        self.ux_data = []
        self.uy_data = []

        fig, ax = plt.subplots()

        self.line1 = ax.plot([],[])

        self.ros_setup()

        self.t_curr = 0
        self.t_offset = 0
        self.first_callback = False

    def ros_setup(self):
        self.state_sub = message_filters.Subscriber('/state', simple_system_state)
        self.input_sub = message_filters.Subscriber('/input', simple_system_control_input)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.state_sub, self.input_sub],
                                                              queue_size = 10,
                                                              slop = 0.1,
                                                              allow_headerless=True)
        self.ts.registerCallback(self.callback_data)
        plt.show()


    def callback_data(self, msg1, msg2):

        if self.first_callback == False:
            self.first_callback = True
            self.t_offset = rospy.Time.now().to_sec() + rospy.Time.now().to_nsec()*1e-9

        self.temp_time = 0
        self.temp_time = rospy.Time.now().to_sec() + rospy.Time.now().to_nsec()*1e-9
        self.t_curr = self.temp_time - self.t_offset

        print("Time: ", self.t_curr)
        self.state_msg[:] = msg1.s[:]
        self.input_msg[:] = msg2.u[:]
        # print("state: ", self.state_msg)
        # print("input: ", self.input_msg)
        self.x_data.append(self.state_msg[0])
        self.y_data.append(self.state_msg[1])

        if len(self.x_data) > 100:
            self.x_data.pop(0)
            self.y_data.pop(0)

    def update(self):
        




if __name__ == '__main__':
    rospy.init_node('plot_node')
    plot_node = PlotNode()
    rospy.spin()
