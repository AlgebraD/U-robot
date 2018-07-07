#!/usr/bin/env python

""" sonar2laser.py - Version 1.1 2013-12-20

    Translate the /sensor_msgs/Range  to  /sensor_msgs/LaserScan ,and publish topic.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

class Sonar2Laser():
    def __init__(self):
        # Give the node a name
        rospy.init_node('sonar2laser', anonymous=False)

        self.sonar_topics = rospy.get_param('~sonar_topics',[])
        self.output_topic = rospy.get_param('~output_topic','sonar2laser')
        self.frame_id = rospy.get_param('~frame_id','sonar2laser')

        rospy.loginfo('sonar_topics:')
        rospy.loginfo(self.sonar_topics)
        rospy.loginfo('output_topic:')
        rospy.loginfo(self.output_topic)
        # Publisher of type nav_msgs/Odometry
        self.laserPub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        
        rospy.loginfo('wait for msg')
        # Wait for the topic to become available
        for topic in self.sonar_topics:
            rospy.wait_for_message(topic, Range)
        
        # Subscribe to the topic
        for topic in self.sonar_topics:
            rospy.Subscriber(topic, Range, self.pub_laser)
        
        rospy.loginfo("Translate Range msg to LaserScan msg")
        
    def pub_laser(self, msg):
        laser = LaserScan()

        laser.header = msg.header
        laser.header.frame_id = self.frame_id
        laser.angle_min = 0
        laser.angle_max = 3.14
        laser.angle_increment = 0.01
        laser.time_increment = 0.01
        laser.scan_time = 0.1
        laser.range_min = 0.2
        laser.range_max = 4.5
        laser.ranges = [msg.range,msg.range]
        laser.intensities = [1,1]

        self.laserPub.publish(laser)
        
if __name__ == '__main__':
    try:
        Sonar2Laser()
        rospy.spin()
    except:
        pass
        

        
