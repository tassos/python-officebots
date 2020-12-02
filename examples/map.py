#!/usr/bin/env python3

import sys
import asyncio

import numpy as np
import cv2

from officebots import Robot


if len(sys.argv) != 2:
    print("Usage: %s <robot name>" % sys.argv[0])
    sys.exit(1)

name = sys.argv[1]


ARROW_UP = 82
ARROW_DOWN = 84
ARROW_RIGHT = 83
ARROW_LEFT = 81
KEY_ESC = 27


class MyRobot(Robot):

    CANVAS_SIZE=1024 #px
    MAP_SIZE=60 #m

    def __init__(self):
        super().__init__()

        self.base_map = np.zeros((self.CANVAS_SIZE,self.CANVAS_SIZE,3), np.uint8)
        self.canvas = None

        self.scale = self.CANVAS_SIZE/self.MAP_SIZE
        self.center = np.array([self.CANVAS_SIZE/2, self.CANVAS_SIZE/2], np.int32)

        print("With map window focused, press Esc to quit, arrows to move the robot.")

    def draw_map(self):
        for polygon in self.map:
            pts = (np.array(polygon)[:,0:2] * self.scale).astype(np.int32) + self.center

            pts = pts.reshape((-1,1,2))
            self.base_map = cv2.fillPoly(self.base_map,[pts],(100,200,200))

    async def on_robot_update(self, data):
        x,y,_,_,_ = data["odom"]
        coordinates = (np.array((x,y)) * self.scale).astype(np.int32) + self.center
        self.canvas = cv2.circle(self.canvas, tuple(coordinates), 5, (0,0,255), -1)

    async def run(self):

        err, self.map = await self.execute(["server", "get-navmesh"])

        self.draw_map()
        self.canvas = self.base_map.copy()

        await self.execute([name, "create"])

        v = old_v = 0.
        w = old_w = 0.


        while True:
            await asyncio.sleep(0.03)
            cv2.imshow('image',self.canvas)
            key = cv2.waitKey(10)

            if key != -1:
                if key == KEY_ESC:
                    self.stop()
                    break
                if key == ARROW_UP:
                    v = 1.0
                if key == ARROW_DOWN:
                    v = -0.8
                if key == ARROW_LEFT:
                    w = 0.8
                if key == ARROW_RIGHT:
                    w = -0.8
            else:
                v = 0
                w = 0

            if (v != old_v) or (w != old_w):
                await self.execute([name, "cmd-vel", [v, w]])
                old_v = v
                old_w = w

MyRobot().start()
cv2.destroyAllWindows()
