#!/usr/bin/env python3

import logging
logger = logging.getLogger(f'rosout.{__name__}')

import sys
import asyncio

from officebots import Robot

officebots_logger = logging.getLogger('officebots')

from math import cos, sin,pi,floor,sqrt

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import tf

def restore_logging():
    officebots_logger.addHandler(logging.StreamHandler())
    officebots_logger.setLevel(logging.DEBUG)

class RosOfficeBots(Robot):

    base_frame = "base_link"

    def __init__(self):
        super().__init__()
        rospy.init_node('officebots')
        restore_logging()
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)

        self.br = tf.TransformBroadcaster()

        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = self.base_frame
        self.odom_msg.pose.pose.position.z = 0.
        self.odom_msg.pose.pose.orientation.x = 0.
        self.odom_msg.pose.pose.orientation.y = 0.

        # laser scan parameters
        self.angle_min = -45. * pi/180
        self.angle_max = 46. * pi/180
        self.angle_increment = 2. * pi/180
        self.range_min = 0. #m
        self.range_max = 60. #m

        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=1)
        self.scan_msg = LaserScan()
        self.scan_msg.header.frame_id = self.base_frame
        self.scan_msg.angle_min = self.angle_min
        self.scan_msg.angle_max = self.angle_max
        self.scan_msg.angle_increment = self.angle_increment
        self.scan_msg.time_increment = 0.
        self.scan_msg.range_min = self.range_min
        self.scan_msg.range_max = self.range_max


    async def run(self):
        logger.info("Starting")
        # simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            await asyncio.sleep(0.1)

        self.stop()
        logger.info("Bye")

    async def on_robot_update(self, data):

        x, y, theta, v, w = data["odom"]

        qx,qy,qz,qw = tf.transformations.quaternion_from_euler(0, 0, theta)

        self.odom_msg.header.stamp = rospy.Time.now() 
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y

        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw

        self.odom_msg.twist.twist.linear.x = v
        self.odom_msg.twist.twist.angular.z = w

        self.odom_pub.publish(self.odom_msg)


        self.br.sendTransform((x, y, 0),
                               (qx,qy,qz,qw),
                               rospy.Time.now(),
                               self.base_frame,
                               "odom")

        #self.scan_msg.scan_time = dt
        #self.scan_msg.ranges = self.ranges

        #self.scan_pub.publish(self.scan_msg)


    def on_cmd_vel(self, twist):
        x = twist.linear.x
        w = twist.angular.z

        self._event_loop.create_task(self.execute(["WallE", "cmd-vel", [x,w]]))

if __name__ == '__main__':
    RosOfficeBots().start()

