#!/usr/bin/env python 

from builtins import range
from builtins import object

import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.transforms as mtransforms
from sensor_msgs.msg import Joy
from opencv_apps.msg import Point2DStamped
from std_msgs.msg import Bool, Float32MultiArray
import rospy


class Plotter(object):

    def __init__(self):

        self.numFig = rospy.get_param('~numFig', 1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.flow_sub = rospy.Subscriber('/flowmean', Point2DStamped, self.flow_cb)
        self.user_display_sub = rospy.Subscriber('/user_display', Float32MultiArray, self.user_display_cb)

        self.correction_pub = rospy.Subscriber('/orientation_correction_on', Bool, self.correction_cb)

        self.ax0 = 0.
        self.ax1 = 0.
        self.u = 0.
        self.v = 0.
        self.arrow = np.array([self.ax0,self.ax1])
        self.speed = np.array([self.u,self.v])
        self.correctionOn = False
        self.r_max = 1.5

        self.speed_des = 0.
        self.speed_meas = 0.
        self.std_des = 0.
        self.std_meas = 0.
        self.min_meas_speed = 0.
        self.min_des_speed = 0.
        self.max_std_meas = 0.


    def flow_cb(self,msg):
        
        #Get speed in image
        self.u = msg.point.x
        self.v = msg.point.y
        self.speed = np.array([self.u,self.v])

    def user_display_cb(self,msg):
        
        self.speed_des = msg.data[0]
        self.speed_meas = msg.data[1]
        self.std_des = msg.data[2]
        self.std_meas = msg.data[3]

        self.min_des_speed = msg.data[4]
        self.min_meas_speed = msg.data[5]
        self.max_std_meas = msg.data[6]


    def joy_cb(self,msg):
        
        #Get axes from joystick
        self.ax0 = -msg.axes[0]
        self.ax1 = msg.axes[1]
        self.arrow = np.array([self.ax0,self.ax1])

    def correction_cb(self,msg):
        
        self.correctionOn = msg.data

    def plotJoystick(self):

        # Get orientation and magnitude
        a_joy = math.atan2(self.ax1,self.ax0)
        mag_joy = np.linalg.norm(self.arrow)

        a_speed = math.atan2(self.v,self.u)


        # Polar plot with arrow
        plt.figure(num=1, figsize=(4, 4))  
        plt.clf()
        ax = plt.subplot(111,polar=True)
        ax.arrow(0, 0, 0, mag_joy, transform=mtransforms.Affine2D().translate(a_joy, 0) + ax.transData, width = 0.1, alpha=0.8,head_width=0.3, head_length=0.2 ,edgecolor = 'steelblue', facecolor = 'lightblue', lw = 1., zorder = 5)
        
        # Check if correction is on
        if self.correctionOn:
            linecolor = 'red'
        else:
            linecolor = 'green'

        # Check norm of speed
        if np.linalg.norm(self.speed) > 0.5:
            mag_speed = self.r_max
            # Check norm of joy and display sector
            if np.linalg.norm(self.arrow) > 0.1:
                da = a_speed - a_joy
                if da > math.pi:
                    da -= 2* math.pi
                if da <  - math.pi:
                    da += 2* math.pi
                ax.fill_between(np.linspace(a_joy, a_joy+da, 50), 0, self.r_max, alpha=0.4, color=linecolor)
        else:
            mag_speed = 0.

        ax.plot((a_speed, a_speed), (mag_speed/50, mag_speed), linewidth = 2, color = linecolor, zorder=5)
        
        ax.set_rmax(self.r_max)
        ax.grid(True)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        plt.draw()
        plt.show(block=False)
        plt.pause(0.005)


    def plotUserDisplay(self):

        # Prepare plot
        plt.figure(num=2, figsize=(4, 4))  
        plt.clf()
        ax_speed_des = plt.subplot(221,polar=True)
        ax_speed_meas = plt.subplot(222,polar=True)
        ax_std_des = plt.subplot(223,polar=True)
        ax_std_meas = plt.subplot(224,polar=True)

        n = 20
        t = np.linspace(0, 2*math.pi, n)

        # Speed des
        line_color = 'red'
        if self.speed_des > self.min_des_speed: line_color = 'green'
        ax_speed_des.plot(t,self.speed_des*np.ones(n),color=line_color)
        ax_speed_des.fill_between(t, self.min_des_speed, 10., alpha=0.4, color='green')
        ax_speed_des.grid(True)
        ax_speed_des.set_xticklabels([])
        ax_speed_des.set(ylim=(0,1.5))
        ax_speed_des.autoscale(False)
        ax_speed_des.title.set_text('Desired speed')

        # Speed meas
        line_color = 'red'
        if self.speed_meas > self.min_meas_speed: line_color = 'green'
        ax_speed_meas.plot(t,self.speed_meas*np.ones(n),color=line_color)
        ax_speed_meas.fill_between(t, self.min_meas_speed, 10., alpha=0.4, color='green')
        ax_speed_meas.grid(True)
        ax_speed_meas.set_xticklabels([])
        ax_speed_meas.set(ylim=(0,1.5))
        ax_speed_meas.autoscale(False)
        ax_speed_meas.title.set_text('Measured speed')

        # std des
        line_color = 'red'
        if self.std_des < self.max_std_meas: line_color = 'green'
        ax_std_des.plot(t,self.std_des*np.ones(n),color=line_color)
        ax_std_des.fill_between(t, 0, self.max_std_meas, alpha=0.4, color='green')
        ax_std_des.grid(True)
        ax_std_des.set_xticklabels([])
        ax_std_des.set(ylim=(0,3.0))
        ax_std_des.autoscale(False)
        ax_std_des.title.set_text('Std des speed')


        # std meas
        line_color = 'red'
        if self.std_meas < self.max_std_meas: line_color = 'green'
        ax_std_meas.plot(t,self.std_meas*np.ones(n),color=line_color)
        ax_std_meas.fill_between(t, 0, self.max_std_meas, alpha=0.4, color='green')
        ax_std_meas.grid(True)
        ax_std_meas.set_xticklabels([])
        ax_std_meas.set(ylim=(0,3.0))
        ax_std_meas.autoscale(False)
        ax_std_meas.title.set_text('Std meas speed')

        plt.draw()
        plt.show(block=False)
        plt.pause(0.005)
 

if __name__ == '__main__':
    rospy.init_node('plotter_node', log_level=rospy.DEBUG)

    node = Plotter()

    r = rospy.Rate(60) # 60hz

    while not rospy.is_shutdown():
        node.plotJoystick()
        # node.plotUserDisplay()
        r.sleep()

    rospy.spin()




