#!/usr/bin/env python3

import sys
import base64

import time

import numpy as np
import cv2

from officebots import Robot


if len(sys.argv) != 2:
    print("Usage: %s <robot name>" % sys.argv[0])
    sys.exit(1)

name = sys.argv[1]


class MyRobot(Robot):
    def load_jpg(self, path):
        with open(path, "rb") as img:
            raw = img.read()

        return base64.b64encode(raw).decode("ascii")

    async def run(self):

        await self.execute([name, "create"])

        while True:
            start = time.time()
            status, png_data = await self.execute([name, "export-camera"])
            got_data = time.time()
            print("Got %s bytes" % len(png_data))
            array = np.asarray(bytearray(png_data), dtype=np.uint8)

            got_img = time.time()

            # with open("/tmp/test.png", "wb") as f:
            #    f.write(array)
            image = cv2.imdecode(array, cv2.IMREAD_COLOR)

            displayed = time.time()

            print(
                "Fetch data: %s; make array: %s; decode: %s"
                % (got_data - start, got_img - got_data, displayed - got_img)
            )
            cv2.imshow("Godot robot", image)

            cv2.waitKey(1)


MyRobot().start()
