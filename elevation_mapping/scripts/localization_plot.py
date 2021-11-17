#!/usr/bin/env python3

from tf import TransformListener
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class Visualizer:
    def __init__(self):
        self.tf = TransformListener()
        self.fig, self.ax = plt.subplots()
        self.ln, = plt.plot([], [], 'ro')
        self.x_data, self.y_data, self.z_data = [] , [], []
        self.z_error = []
        self.t_data = []
        self.start_time = rospy.Time.now().to_sec()
        #print(self.start_time)
        #print(type(self.start_time.to_sec()))
        self.latest_time = None

        self.trans = 0
        # self.location_x = 0
        # self.location_y = 0
        # self.location_z = 0

        self.true_z = 0
    
    def plot_init(self):
        
        return self.ln
    
    def get_tracking_tf(self):
        #if self.tf.frameExists('cam_tracking_pose_frame') and self.tf.frameExists('map'):
        try:
            t = rospy.Time(0)
            (trans, rot) = self.tf.lookupTransform("map","cam_depth_link",  t)
            print(trans)
            self.trans = trans
            self.latest_time = rospy.Time.now().to_sec()


        except:
            rospy.loginfo('tf not available')
            return 

        self.set_data()    
        
    def set_data(self):
        self.x_data.append(self.trans[0])
        self.y_data.append(self.trans[1])
        self.z_data.append(self.trans[2])
        self.z_error.append(self.true_z - self.trans[2])
        duration = self.latest_time - self.start_time
        self.t_data.append(duration)
        #print(self.y_data)
        list_len = 200
        if len(self.x_data) > list_len:
            self.x_data.pop(0)
        if len(self.y_data) > list_len:
            self.y_data.pop(0)
        if len(self.z_data) > list_len:
            self.z_data.pop(0)
        if len(self.z_error) > list_len:
            self.z_error.pop(0)
        if len(self.t_data) > list_len:
            self.t_data.pop(0)

    # def update_plot(self, frame):

    #     self.ln.set_data(self.t_data, self.y_data)
    #     return self.ln

    def plot(self, direction):
        plt.clf()
        plt.ylim([-0.001, 0.001])
        plt.ylabel('error from truth (m)')
        # plt.xlabel('time from start (s)')
        # plt.axhline(y=0, color='r', linestyle='-')
        if direction =='x':
            plt.plot(self.t_data, self.x_data)
        if direction =='y':
            plt.plot(self.t_data, self.y_data)
        if direction =='z':
            plt.plot(self.t_data, self.z_data)
        if direction =='z_error':
            plt.plot(self.t_data, self.z_error)

rospy.init_node('localization_plot')
vis = Visualizer()
while not rospy.is_shutdown():

    vis.get_tracking_tf()
    vis.plot('z')
    plt.pause(0.01)
    # ani = FuncAnimation(vis.fig,vis.update_plot, init_func=vis.plot_init)

# plt.show(block=True)

#rospy.spin()

