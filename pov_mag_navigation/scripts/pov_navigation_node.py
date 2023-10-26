#!/usr/bin/env python

from __future__ import print_function
from builtins import object

import rospy
import math
import numpy as np
import tf
from matplotlib import pyplot as plt
from numpy.linalg import inv as inv
from numpy.linalg import svd as  svd
from opencv_apps.msg import FlowArrayStamped, Flow, Point2D, Point2DStamped
from std_msgs.msg import Float32, Float32MultiArray, Int32, Bool
from mag_msgs.msg import FieldStamped
from std_srvs.srv import Trigger, TriggerResponse
from tsc_utils.conversions import np_to_vector3_msg
from dynamic_reconfigure.server import Server
from pov_mag_navigation.cfg import PovNavConfig
from mag_rod_msgs.msg import CatheterState
from mag_rod_msgs.msg import LengthStamped


class PovNavigation(object):
    
    def __init__(self):

        
        self.length = 0.

        flow_sub = rospy.Subscriber("/flow/shift", Point2DStamped, self.flow_cb)
        speed_sub = rospy.Subscriber("speed_des", Point2D, self.speed_cb)
        field_change_sub = rospy.Subscriber("change_field_mag", Int32, self.field_change_cb)
        cath_state_sub = rospy.Subscriber("/const_curv_sim/catheter_state", CatheterState, self.cath_state_cb)
        insertion_sub = rospy.Subscriber("/insert_speed", Float32, self.insertion_cb)
        
        self.field_pub = rospy.Publisher('/desired_field', FieldStamped, queue_size=1)
        self.flow_pub = rospy.Publisher('/flowmean', Point2DStamped, queue_size=1)
        self.speed_sim_pub = rospy.Publisher('speed_sim', Point2DStamped, queue_size=1)
        self.correction_pub = rospy.Publisher('/orientation_correction_on', Bool, queue_size=1)
        self.user_display_pub = rospy.Publisher('/user_display', Float32MultiArray, queue_size=1)
        self.insertion_length_pub = rospy.Publisher('/mca/desired_length', LengthStamped, queue_size=1)

        self.tf_broadcaster = tf.TransformBroadcaster()
        
        calib_srv = rospy.Service('do_calib', Trigger, self.doCalibration)
        save_config_srv = rospy.Service('save_config', Trigger, self.saveConfig)
        goto_config_srv = rospy.Service('goto_config', Trigger, self.gotoConfig)
        
        # Param
        self.rate = rospy.get_param('~rate', 30)
        print(self.rate)
        self.max_rot_speed = math.pi #rad/s
        self.field_mag_max = 0.02 #T
        self.field_mag_inc = 0.005 #T
        self.field_mag = 0.015 #T
        self.start_el = math.pi/2 #math.pi/4 + math.pi
        self.start_az = 0. #math.pi/4 + math.pi
        self.Tavg = 0.5
        N = int(self.Tavg * self.rate)
        self.A = 10.
        self.now = rospy.Time.now().to_sec()
        self.do_update_jacobian = rospy.get_param('~do_update_jacobian', True)

        # Simuate the Jacobian without camera and cmag
        self.sim_on = False

        # Parameter calibration
        # Ampl calibration (rad)
        self.amp = math.pi/8

        # Duration calibration (s)
        self.Tcal = 1.5
        self.Twait = 0.5

        # Measure in the image
        self.um = 0.
        self.vm = 0.
        # Rotation speed
        self.alpha1 = 0.
        self.alpha2 = 0.

        # Covariance matrices
        self.M = 1.e-3 * np.eye(4,dtype= float)
        self.N = 1. * np.eye(2,dtype= float)

        # Initialiazation of Jacobian
        aa = 0. #math.pi/4
        gain = 70. #px/rad
        self.R = np.array([[math.cos(aa), -math.sin(aa)],[math.sin(aa), math.cos(aa)]])
        self.Rsim = self.R 
        self.J0 = np.matmul(self.R,np.array([[0., -gain],[-gain , 0.]]))
        self.Jest = self.J0
        self.J_saved = self.J0

        # Initialiazation for KF
        self.Xk = np.reshape(self.J0,(4,1))
        self.Yk = np.array([[self.um],[self.vm]])
        self.sigk = np.zeros((4,4))
        self.Ck = np.array([[self.alpha1, self.alpha2, 0., 0.],[0.,0.,self.alpha1, self.alpha2]])

        # Initialization for field
        self.field_axis = np.array([[1.],[0.],[0.]])
        # With elevation azimuth
        self.el = self.start_el
        self.az = self.start_az
        # With roll and pitch

        # Initialization optical flow
        self.um_avg = np.zeros((N,1))
        self.vm_avg = np.zeros((N,1))
        self.uref_avg = np.zeros((N,1))
        self.vref_avg = np.zeros((N,1))

        self.min_meas_speed = 1.
        self.min_des_speed = 0.1
        self.max_std_meas = 0.1

        self.speed_des = 0.
        self.speed_meas = 0.
        self.std_des = 0.
        self.std_meas = 0.

        self.u_des = 0.
        self.v_des = 0.
        self.u_des_filt = 0.
        self.v_des_filt = 0.

        # Initilialize transform
        self.q  = tf.transformations.quaternion_from_euler(0., self.start_el, self.start_az)
        self.q_init = self.q
        self.q_saved = self.q

        self.tf_endoscope_mns = tf.TransformBroadcaster()

        self.tf_endoscope_mns.sendTransform((0., 0., 0.),
                     (self.q[0], self.q[1], self.q[2], self.q[3]),
                     rospy.Time.now(),
                     "endoscope",
                     "mns")

        # Initialize system
        self.srv = Server(PovNavConfig, self.ConfigCallback)

        self.calibration_on = False
        self.doCorrection = False
        self.angle_correction = 0.
        self.angle_treshold = math.pi/6

        self.init()

        # self.doCalibration(True)
        self.i = 0
        self.mainTask()


    def cath_state_cb(self,msg):
        """
        Broadcasts frame at the destal end of
        the scope simulated by the const_curv_sim.
        package."""

        cath_state = msg
        
        self.tf_broadcaster.sendTransform((
            cath_state.p_x[-1],
            cath_state.p_y[-1],
            cath_state.p_z[-1]),
           (cath_state.q_x[-1], 
            cath_state.q_y[-1],
            cath_state.q_z[-1],
            cath_state.q_w[-1]),
            rospy.Time.now(),
            "distal",
            "mns")


    def insertion_cb(self,msg):
        length_incr = msg.data
        self.length = self.length + length_incr

        length = LengthStamped()
        length.length = self.length
        self.insertion_length_pub.publish(length)

    
    def ConfigCallback(self,config, level):

    
        if self.Tavg != config['time_average']:
            self.Tavg = config['time_average']
            N = int(self.Tavg * self.rate)
            self.um_avg = np.zeros((N,1))
            self.vm_avg = np.zeros((N,1))
            self.uref_avg = np.zeros((N,1))
            self.vref_avg = np.zeros((N,1))


        self.min_meas_speed = config['min_meas_speed']
        self.min_des_speed = config['min_des_speed']
        self.max_std_meas = config['max_std_meas']
        self.Tcal = config['time_to_calibrate']
        self.sim_on = config['sim_on']
        self.angle_treshold = config['angle_treshold'] * math.pi / 180.
        self.A = config['exponential_factor']

        if self.sim_on:
            self.u_des = config['u_des_sim']
            self.v_des = config['v_des_sim']
            speed = Point2DStamped()
            speed.header.stamp = rospy.Time.now()
            speed.point.x = self.u_des
            speed.point.y = self.v_des
            self.speed_sim_pub.publish(speed)
            aa = config['simulated_angle'] * math.pi / 180.
            self.Rsim = np.array([[math.cos(aa), -math.sin(aa)],[math.sin(aa), math.cos(aa)]])

        return config

    
    def init(self):

        r = rospy.Rate(self.rate)

        self.q = self.q_init

        T = rospy.Time.now().to_sec()
        Tf = T + 2.

        print('=== Init field ===')
        while T < Tf and not rospy.is_shutdown(): 
            
            if not (self.calibration_on):
                self.sendField()

            T = rospy.Time.now().to_sec()
            r.sleep()


        print('=== end init field ===')


    def emulateSys(self):

        if self.sim_on:
            gain_u = 100. #px/rad
            gain_v = 100. #px/rad
            J = np.matmul(self.Rsim,np.array([[0., -gain_u],[-gain_v , 0.]]))
            # print('=== Virtual Jacobian ===')
            # print(J)

            aa = math.pi/4
            R = np.array([[math.cos(aa), -math.sin(aa)],[math.sin(aa), math.cos(aa)]])
            J = np.matmul(R,np.array([[10., 0.],[0. , 50.]]))
            u, s, vh = svd(J, full_matrices=True)
            print(u)
            print(s)
            print(vh)

            # Extract flow and use running average
            self.um_avg = np.roll(self.um_avg, 1, axis = None)
            self.vm_avg = np.roll(self.vm_avg, 1, axis = None)

            alpha = np.array([[self.alpha1],[self.alpha2]])
            v = J.dot(alpha)
            
            self.um_avg[0] = v[0,0]
            self.vm_avg[0] = v[1,0]

            row, col = self.um_avg.shape
            self.um = np.squeeze(sum(self.um_avg)/row)
            self.vm = np.squeeze(sum(self.vm_avg)/row)

            flow = Point2DStamped()
            flow.header.stamp = rospy.Time.now()
            flow.point.x = self.um
            flow.point.y = self.vm
            self.flow_pub.publish(flow)

            speed = Point2DStamped()
            speed.header.stamp = rospy.Time.now()
            speed.point.x = self.u_des
            speed.point.y = self.v_des
            self.speed_sim_pub.publish(speed)
    
    
    def flow_cb(self,msg):

        if not self.sim_on:
            # Extract flow and use running average
            self.um_avg = np.roll(self.um_avg, 1, axis = None)
            self.vm_avg = np.roll(self.vm_avg, 1, axis = None)

            self.um_avg[0] = -msg.point.x
            self.vm_avg[0] = msg.point.y

            row, col = self.um_avg.shape
            self.um = np.squeeze(sum(self.um_avg)/row)
            self.vm = np.squeeze(sum(self.vm_avg)/row)

            flow = Point2DStamped()
            flow.header.stamp = rospy.Time.now()
            flow.point.x = self.um
            flow.point.y = self.vm
            self.flow_pub.publish(flow)

    def average_speed(self):

        self.uref_avg = np.roll(self.uref_avg, 1, axis = None)
        self.vref_avg = np.roll(self.vref_avg, 1, axis = None)

        self.uref_avg[0] = self.u_des
        self.vref_avg[0] = self.v_des

        row, col = self.um_avg.shape
        self.u_des_filt = np.squeeze(sum(self.uref_avg)/row)
        self.v_des_filt = np.squeeze(sum(self.vref_avg)/row)

    def speed_cb(self,msg):

        self.u_des = msg.x
        self.v_des = msg.y

    def field_change_cb(self,msg):

        field_unsat = self.field_mag + (float(msg.data) * self.field_mag_inc)

        if field_unsat >= self.field_mag_max:
            field_sat = self.field_mag_max
        elif field_unsat <= 0.:
            field_sat = 0.
        else:
            field_sat = field_unsat
        print("Field changed to:  %.4f" % field_sat)

        self.field_mag = field_sat


    def computeControl(self):

        # Update target field with desired image velocity
        v = np.array([[self.u_des],[self.v_des]])

        # Compute desired angles
        alpha_des = (inv(np.squeeze(self.Jest)).dot(v))

        # Saturate rotation speed
        if abs(alpha_des[0,0]) > (self.max_rot_speed/self.rate):
            self.alpha1 = (self.max_rot_speed * alpha_des[0,0]/abs(alpha_des[0,0]))/self.rate
        else:
            self.alpha1 = alpha_des[0,0]

        if abs(alpha_des[1,0]) > (self.max_rot_speed/self.rate):
            self.alpha2 = (self.max_rot_speed * alpha_des[1,0]/abs(alpha_des[1,0]))/self.rate
        else:
            self.alpha2 = alpha_des[1,0]

        self.sendField()


    def sendField(self):

        # Emulate the system
        self.emulateSys()

        # Update angles
        self.el = self.alpha1 
        self.az = self.alpha2

        # Update frames transform

        dq = tf.transformations.quaternion_from_euler(0., self.el, self.az)
        self.q = tf.transformations.quaternion_multiply(self.q, dq)

        self.tf_endoscope_mns.sendTransform((0., 0., 0.),
                     (self.q[0], self.q[1], self.q[2], self.q[3]),
                     rospy.Time.now(),
                     "endoscope",
                     "mns")

        # Publish field
        field_msg = FieldStamped()
        field_msg.header.stamp = rospy.Time.now()
        field_msg.header.frame_id = 'endoscope'
        field_msg.field.vector = np_to_vector3_msg(self.field_axis * self.field_mag)
        self.field_pub.publish(field_msg)


    def mainTask(self):

        r = rospy.Rate(self.rate)

        while not rospy.is_shutdown(): 

            self.average_speed()
            
            if not (self.calibration_on):
                # Update the Jacobian
                if self.do_update_jacobian:
                    self.updateJacobian_method_Boehler()
                    ### Outdated methods ###
                    # self.updateJacobian_method_Edelman()
                    # self.updateJacobian_method_Zhang()

                # Compute control and send field
                self.computeControl()

            # broadcast mobile world frame for insertion
            self.tf_broadcaster.sendTransform((
                0,0,self.length),
                (0,0,0,1),
                rospy.Time.now(),
                "world_mob",
                "world")

            r.sleep()


    def doCalibration(self,req):

        self.calibration_on = True

        v_cal1 = np.array([], dtype=float).reshape(2,0)
        v_cal2 = np.array([], dtype=float).reshape(2,0)

        r = rospy.Rate(self.rate)

        # Prepare for cal 1
        T = rospy.Time.now().to_sec()
        Tf = T + self.Tcal
        print("Calibration: phase 1...")
        while T < Tf and not rospy.is_shutdown():
            self.alpha1 = (self.amp/self.Tcal)/self.rate
            self.alpha2 = 0.
            #Send field
            self.sendField()
            # Collect motion
            if (Tf-T ) < 0.9*self.Tcal:
                v_cal1 =np.hstack((v_cal1, np.array([[self.um],[self.vm]])))

            T = rospy.Time.now().to_sec()
            r.sleep()
        dp1 = np.array([[self.alpha1],[self.alpha2]])

        T = rospy.Time.now().to_sec()
        Tf = T + self.Tcal
        print("Return to position...")
        while T < Tf and not rospy.is_shutdown():
            self.alpha1 = -(self.amp/self.Tcal)/self.rate
            self.alpha2 = 0.
            #Send field
            self.sendField()
            T = rospy.Time.now().to_sec()
            r.sleep()

        rospy.sleep(1.) 

        # Prepare for cal 2
        T = rospy.Time.now().to_sec()
        Tf = T + self.Tcal
        print("Calibration: phase 2...")
        while T < Tf and not rospy.is_shutdown(): 
            self.alpha1 = 0.
            self.alpha2 = (self.amp/self.Tcal)/self.rate
            # Send field
            self.sendField()
            # Collect motion
            if (Tf-T ) < 0.9*self.Tcal:
                v_cal2 =np.hstack((v_cal2, np.array([[self.um],[self.vm]])))


            T = rospy.Time.now().to_sec()
            r.sleep()
        dp2 = np.array([[self.alpha1],[self.alpha2]])

        T = rospy.Time.now().to_sec()
        Tf = T + self.Tcal
        print("Return to position...")
        while T < Tf and not rospy.is_shutdown():
            self.alpha1 = 0.
            self.alpha2 = -(self.amp/self.Tcal)/self.rate
            #Send field
            self.sendField()
            T = rospy.Time.now().to_sec()
            r.sleep()


        # Data to build Jacobian
        row, col = v_cal1.shape
        dv1 = np.reshape(np.sum(v_cal1, axis=1)/col,(2,1))
  
        row, col = v_cal2.shape
        dv2 = np.reshape(np.sum(v_cal2, axis=1)/col,(2,1))

        # Initial estimate of jacobian
        self.J0 = np.matmul(np.hstack((dv1, dv2)),inv(np.hstack((dp1, dp2))))
        self.Jest = self.J0
        self.Xk = np.reshape(self.J0,(4,1))
        
        print("=== Estimated Jacobian after calibration ====")
        print(self.J0)

        self.calibration_on = False

        return TriggerResponse(success=True, message="calib successfull")


    def gotoConfig(self,req):

        self.q = self.q_saved
        self.Jest = self.J_saved

        return TriggerResponse(success=True, message="goto successfull")


    def saveConfig(self,req):

        self.q_saved = self.q
        self.J_saved = self.Jest

        return TriggerResponse(success=True, message="goto successfull")
   


    def updateJacobian_method_Boehler(self):

        # Check parameters
        self.speed_des = np.linalg.norm(np.array([[self.u_des_filt],[self.v_des_filt]]))
        # self.speed_des = np.linalg.norm(np.array([[self.u_des],[self.v_des]]))
        self.speed_meas = np.linalg.norm(np.array([[self.um],[self.vm]]))
        self.std_des = np.maximum(np.std(self.uref_avg),np.std(self.vref_avg))
        self.std_meas = np.maximum(np.std(self.um_avg),np.std(self.vm_avg))

        usr_disp_msg = Float32MultiArray()
        data = [self.speed_des, self.speed_meas, self.std_des, self.std_meas, self.min_des_speed, self.min_meas_speed, self.max_std_meas]

        usr_disp_msg.data = data
        self.user_display_pub.publish(usr_disp_msg)

        # Check if the velocities are high and stable enough
        if (self.speed_des > self.min_des_speed and self.speed_meas > self.min_meas_speed and self.std_meas < self.max_std_meas and self.std_des < self.max_std_meas):

            if not self.doCorrection:
                # Prepare velocity vectors
                # u_ref = np.array([[self.u_des],[self.v_des]])
                u_ref = np.array([[self.u_des_filt],[self.v_des_filt]]) #used averaged desired speed
                u_meas = np.array([[self.um], [self.vm]])

                # Normalization
                u_ref_n = np.reshape(u_ref/np.linalg.norm(u_ref),(2,1))
                u_meas_n = np.reshape(u_meas/np.linalg.norm(u_meas),(2,1))
                
                # Measure rotation
                a_meas = math.atan2(u_meas_n[1],u_meas_n[0])
                a_ref = math.atan2(u_ref_n[1],u_ref_n[0])
                da = a_meas - a_ref

                if da > math.pi:
                    da -= 2* math.pi
                if da <  - math.pi:
                    da += 2* math.pi

                print("Angle difference:  %.1fdeg (threshold is %.1fdeg)" % ((da * 180./math.pi), (self.angle_treshold * 180./math.pi)))

                # Trigger the correction if angle difference is superior to a treshold
                if abs(da) > self.angle_treshold:
                    self.doCorrection = True
                    self.angle_correction = da
                    print("Correction...")

        # Perform angle correction with exponential decay profile
        if self.doCorrection:
            # Exponential decay profile for correction
            aa = self.angle_correction/self.A
            # print("Correction:  %.6fdeg" % (aa * 180./math.pi))
            self.R = np.array([[math.cos(aa), -math.sin(aa)],[math.sin(aa), math.cos(aa)]])
            self.Jest = np.matmul(self.R,self.Jest)
            self.angle_correction = self.angle_correction - aa

            if abs(self.angle_correction) < math.pi/30:
                self.doCorrection = False
        
        correction_msg = Bool()
        correction_msg.data = self.doCorrection
        self.correction_pub.publish(correction_msg)

        # Update gain
        # dg = np.log(np.linalg.norm(u_meas)/np.linalg.norm(u_ref))
        # if abs(dg) > 0.1:
        #     self.Jest = np.exp(dg/A) * self.Jest

        # if abs(self.u_des) > 1.:
        #     dg_u = np.log(abs(self.um)/abs(self.u_des))
        #     if abs(dg_u) > 0.1:
        #         self.Jest[0,:] = np.exp(dg_u/A) * self.Jest[0,:]

        # if abs(self.v_des) > 1.:
        #     dg_v = np.log(abs(self.vm)/abs(self.v_des))
        #     if abs(dg_v) > 0.1:
        #         self.Jest[1,:] = np.exp(dg_v/A) * self.Jest[1,:]


        # print('=== Estimated Jacobian ===')
        # print(self.Jest)

    def updateJacobian_method_Edelman(self):

        if (np.linalg.norm(np.array([[self.u_des],[self.v_des]])) > self.min_meas_speed) and (np.linalg.norm(np.array([[self.um],[self.vm]])) > self.min_meas_speed) and np.std(self.um_avg) < self.max_std_meas and np.std(self.vm_avg) < self.max_std_meas:

            #Update inputs and states
            self.Yk[0] = self.um
            self.Yk[1] = self.vm
            self.Ck = np.array([[self.alpha1, self.alpha2, 0., 0.],[0.,0.,self.alpha1, self.alpha2]])
            Ckt = np.transpose(self.Ck)

            #Update iteration
            self.Xk_1 = self.Xk
            self.sigk_1 = self.sigk
            
            #Perform KF
            sigk_tild = self.sigk_1 + self.M

            A = (self.Ck.dot(sigk_tild)).dot(Ckt)
            K = np.matmul(sigk_tild.dot(Ckt),inv(A + self.N))

            B = np.matmul(K.dot(self.Ck),sigk_tild)
            self.sigk = sigk_tild + B

            self.Xk = self.Xk_1 + K.dot(self.Yk - self.Ck.dot(self.Xk_1))
            self.Jest = np.reshape(self.Xk,(2,2))

            print('=== Estimated Jacobian ===')
            print(self.Jest)


    def updateJacobian_method_Zhang(self):

        if (np.linalg.norm(np.array([[self.u_des],[self.v_des]])) > self.min_meas_speed) and (np.linalg.norm(np.array([[self.um],[self.vm]])) > self.min_meas_speed) and np.std(self.um_avg) < self.max_std_meas and np.std(self.vm_avg) < self.max_std_meas:

            dp = np.array([[self.alpha1], [self.alpha2]])
            ds = np.array([[self.um], [self.vm]])

            A = np.matmul(dp,np.transpose(ds))
            B = ds - np.matmul(self.Jest,dp)

            w_tild = np.matmul(A,B)/(((np.linalg.norm(ds))**2.) * ((np.linalg.norm(dp))**2.))

            self.Jest = self.Jest + np.matmul(ds, np.transpose(w_tild))
            
            print('=== Estimated Jacobian ===')
            print(self.Jest)
        


if __name__ == '__main__':
    rospy.init_node('pov_navigation_node', log_level=rospy.INFO)

    try:
        node = PovNavigation()
    except rospy.ROSInterruptException:
        pass