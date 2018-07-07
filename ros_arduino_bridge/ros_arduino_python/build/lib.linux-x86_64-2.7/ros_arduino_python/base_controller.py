#!/usr/bin/env python

"""
    A base controller class for the Arduino microcontroller
    
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
import os

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
 
""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, arduino, base_frame, name="base_controllers"):
        self.arduino = arduino
        self.name = name
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        pid_params['encoder_resolution_A'] = rospy.get_param("~encoder_resolution_A", "") 
	pid_params['encoder_resolution_B'] = rospy.get_param("~encoder_resolution_B", "") 
	pid_params['encoder_resolution_C'] = rospy.get_param("~encoder_resolution_C", "") 
        pid_params['gear_reduction_A'] = rospy.get_param("~gear_reduction_A", 1.0)
        pid_params['gear_reduction_B'] = rospy.get_param("~gear_reduction_B", 1.0)
        pid_params['gear_reduction_C'] = rospy.get_param("~gear_reduction_C", 1.0)

	pid_params['AWheel_Kp'] = rospy.get_param("~AWheel_Kp", 20)
        pid_params['AWheel_Kd'] = rospy.get_param("~AWheel_Kd", 12)
        pid_params['AWheel_Ki'] = rospy.get_param("~AWheel_Ki", 0)
        pid_params['AWheel_Ko'] = rospy.get_param("~AWheel_Ko", 50)

	pid_params['BWheel_Kp'] = rospy.get_param("~BWheel_Kp", 20)
        pid_params['BWheel_Kd'] = rospy.get_param("~BWheel_Kd", 12)
        pid_params['BWheel_Ki'] = rospy.get_param("~BWheel_Ki", 0)
        pid_params['BWheel_Ko'] = rospy.get_param("~BWheel_Ko", 50)

	pid_params['CWheel_Kp'] = rospy.get_param("~CWheel_Kp", 20)
        pid_params['CWheel_Kd'] = rospy.get_param("~CWheel_Kd", 12)
        pid_params['CWheel_Ki'] = rospy.get_param("~CWheel_Ki", 0)
        pid_params['CWheel_Ko'] = rospy.get_param("~CWheel_Ko", 50)
        
	self.linear_scale_correction = rospy.get_param('~linear_scale_correction',1);
	self.angular_scale_correction = rospy.get_param('~angular_scale_correction',1);
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
        self.motors_reversed = rospy.get_param("~motors_reversed", False)
        
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter_A = self.encoder_resolution_A * self.gear_reduction_A  / (self.wheel_diameter * pi)
        self.ticks_per_meter_B = self.encoder_resolution_B * self.gear_reduction_B  / (self.wheel_diameter * pi)
        self.ticks_per_meter_C = self.encoder_resolution_C * self.gear_reduction_C  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel_A = self.accel_limit * self.ticks_per_meter_A / self.rate
        self.max_accel_B = self.accel_limit * self.ticks_per_meter_B / self.rate
	self.max_accel_C = self.accel_limit * self.ticks_per_meter_C / self.rate    
    
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
	self.enc_A = None            # encoder readings
        self.enc_B = None
	self.enc_C = None

        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians

        self.v_A = 0
        self.v_B = 0
	self.v_C = 0

        self.v_des_AWheel = 0             # cmd_vel setpoint
        self.v_des_BWheel = 0
	self.v_des_CWheel = 0

        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        
        # Clear any old odometry info
        self.arduino.reset_encoders()
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.AWheel_Kp = pid_params['AWheel_Kp']
        self.AWheel_Kd = pid_params['AWheel_Kd']
        self.AWheel_Ki = pid_params['AWheel_Ki']
        self.AWheel_Ko = pid_params['AWheel_Ko']

        self.BWheel_Kp = pid_params['BWheel_Kp']
        self.BWheel_Kd = pid_params['BWheel_Kd']
        self.BWheel_Ki = pid_params['BWheel_Ki']
        self.BWheel_Ko = pid_params['BWheel_Ko']

        self.CWheel_Kp = pid_params['CWheel_Kp']
        self.CWheel_Kd = pid_params['CWheel_Kd']
        self.CWheel_Ki = pid_params['CWheel_Ki']
        self.CWheel_Ko = pid_params['CWheel_Ko']
        
	
        self.arduino.update_pid(self.AWheel_Kp, self.AWheel_Kd, self.AWheel_Ki, self.AWheel_Ko,
				self.BWheel_Kp, self.BWheel_Kd, self.BWheel_Ki, self.BWheel_Ko,
				self.CWheel_Kp, self.CWheel_Kd, self.CWheel_Ki, self.CWheel_Ko)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            # Read the encoders
            try:
                A_enc, B_enc , C_enc = self.arduino.get_encoder_counts()
            except:
                self.bad_encoder_count += 1
                rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
                return
                            
            dt = now - self.then
            self.then = now
            dt = dt.to_sec()
            
            # Calculate odometry
            if self.enc_A == None and self.enc_B == None and self.enc_C == None:
                dA = 0
                dB = 0
		dC = 0
            else:
                dA = (A_enc - self.enc_A) / self.ticks_per_meter_A
                dB = (B_enc - self.enc_B) / self.ticks_per_meter_B
                dC = (C_enc - self.enc_C) / self.ticks_per_meter_C

            self.enc_A = A_enc
            self.enc_B = B_enc
            self.enc_C = C_enc

            va = dA / dt
            vb = dB / dt            
            vc = dC / dt

            vx = sqrt(3)*(va-vb)/3
            vy = (va+vb-2*vc)/3
            vth = (va+vb+vc)/(3*self.wheel_track)
            delta_x = (vx*cos(self.th) - vy*sin(self.th))*dt
            delta_y = (vx*sin(self.th) - vy*cos(self.th))*dt
            delta_th = vth*dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            dxy_ave = (dright + dleft) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt

            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)
    
            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                "odom"
                )
    
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.child_frame_id = self.base_frame
            odom.header.stamp = now
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vth

            self.odomPub.publish(odom)
            
            if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):
                self.v_des_AWheel = 0
                self.v_des_BWheel = 0
                self.v_des_CWheel = 0    
            
            if self.v_A < self.v_des_AWheel:
                self.v_A += self.max_accel_A
                if self.v_A > self.v_des_AWheel:
                    self.v_A = self.v_des_AWheel
            else:
                self.v_A -= self.max_accel_A
                if self.v_A < self.v_des_AWheel:
                    self.v_A = self.v_des_AWheel
            
            if self.v_B < self.v_des_BWheel:
                self.v_B += self.max_accel_B
                if self.v_B > self.v_des_BWheel:
                    self.v_B = self.v_des_BWheel
            else:
                self.v_B -= self.max_accel_B
                if self.v_B < self.v_des_BWheel:
                    self.v_B = self.v_des_BWheel
            
            if self.v_C < self.v_des_CWheel:
                self.v_C += self.max_accel_C
                if self.v_C > self.v_des_CWheel:
                    self.v_C = self.v_des_CWheel
            else:
                self.v_C -= self.max_accel_C
                if self.v_C < self.v_des_CWheel:
                    self.v_C = self.v_des_CWheel
            
            # Set motor speeds in encoder ticks per PID loop
            if not self.stopped:
                self.arduino.drive(self.v_A, self.v_B ,self.v_C)
                
            self.t_next = now + self.t_delta
            
    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0,0)
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        y = req.linear.y
        th = req.angular.z       # rad/s

        tmpx = sqrt(3)/2.0
        tmpy = 0.5
        self.v_A = (tmpx*x + tmpy*y + self.wheel_track*self.th)
        self.v_B = (-tmpx*x + tmpy*y + self.wheel_track*self.th)
        self.v_C = (-y + self.wheel_track*self.th)

            
        self.v_des_AWheel = int(self.v_A * self.ticks_per_meter_A / self.arduino.PID_RATE)
        self.v_des_BWheel = int(self.v_B * self.ticks_per_meter_B / self.arduino.PID_RATE)
        self.v_des_CWheel = int(self.v_C * self.ticks_per_meter_C / self.arduino.PID_RATE)        

        

    

    
