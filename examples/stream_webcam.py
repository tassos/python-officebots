#!/usr/bin/env python3

import sys
import base64


import numpy as np
import cv2

from officebots import Robot


if len(sys.argv) != 2:
    print("Usage: %s <robot name>" % sys.argv[0])
    sys.exit(1)

name = sys.argv[1]

cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 380)


class MyRobot(Robot):

    def load_jpg(self, path):
        with open(path, 'rb') as img:
            raw = img.read()

        return base64.b64encode(raw).decode("ascii")

    async def run(self):

        await self.execute([name, "create"])

        while True:
            ret, frame = cap.read()
            if frame is None:
                continue
            frame = cv2.resize(frame, (320, 240))
            ret, buf = cv2.imencode(".jpg", frame)
            b64 = base64.b64encode(buf).decode("ascii")
            print("Image size: %skb" % (len(b64)/1024))

            print("Sending image")
        
            await self.execute([name, "set-screen", [b64]])

MyRobot().start()

cap.release()
