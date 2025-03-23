#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
@作者: 古月居(www.guyuehome.com)
@说明: 视觉巡线控制(gazebo)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

def image_callback(msg):
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    hsv = cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([ 10,  10,  10])
    upper_yellow = np.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image_input.shape
    search_top = int(3*h/4)
    search_bot = int(3*h/4 + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image_input, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      twist = Twist()
      twist.linear.x = 0.2
      twist.angular.z = -float(err) / 500
      publisher.publish(twist)
      # END CONTROL
    cv2.imshow("detect_line", image_input)
    cv2.waitKey(3)


def main():
    rclpy.init()
    global node
    node = Node('follower')

    global publisher
    publisher = node.create_publisher(Twist, '/cmd_vel', rclpy.qos.qos_profile_system_default)
    subscription = node.create_subscription(Image, 'camera/image_raw',
                                            image_callback,
                                            rclpy.qos.qos_profile_sensor_data)

    rclpy.spin(node)

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    publisher.publish(empty_message)

    node.destroy_node()
    rclpy.shutdown()
    exit()
