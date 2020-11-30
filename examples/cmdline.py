#!/usr/bin/env python3

import logging
logging.basicConfig(level=logging.INFO)

import sys

# libraries to load and encode JPG images
import pathlib
import base64

from officebots import Robot

logging.getLogger('officebots').setLevel(logging.DEBUG)

if len(sys.argv) < 3:
    print("Usage: %s <robot name> <cmd> [<param1> <param2> ...]" % sys.argv[0])
    sys.exit(1)



cmd = [sys.argv[1], sys.argv[2], [x for x in sys.argv[3:]]]


class MyRobot(Robot):

    def load_jpg(self, path):
        with open(path, 'rb') as img:
            raw = img.read()

        return base64.b64encode(raw).decode("ascii")

    async def run(self):

        # if calling 'set-screen', we must first load the jpg image
        # and encode it in base64
        if cmd[1] == "set-screen":
            cmd[2] = [self.load_jpg(cmd[2][0])]

        response = await self.execute(cmd)

        print(response)

        self.stop()

MyRobot().start()
