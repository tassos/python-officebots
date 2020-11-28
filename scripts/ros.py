#!/usr/bin/env python3

import logging
logger = logging.getLogger(f'rosout.{__name__}')

import sys
import asyncio

from officebots import Robot

logging.getLogger('officebots').setLevel(logging.DEBUG)

import rospy
from geometry_msgs.msg import Twist


class RosOfficeBots(Robot):

    def __init__(self):
        super().__init__()
        rospy.init_node('officebots')
        rospy.Subscriber("cmd_vel", Twist, self.on_cmd_vel)
        pass

    async def run(self):
        logger.info("Starting")
        # simply keeps python from exiting until this node is stopped
        while not rospy.is_shutdown():
            await asyncio.sleep(0.1)

        self.stop()
        logger.info("Bye")

    def on_cmd_vel(self, twist):
        x = twist.linear.x
        w = twist.angular.z

        self._event_loop.create_task(self.execute(["WallE", "cmd-vel", [x,w]]))

if __name__ == '__main__':
    RosOfficeBots().start()

