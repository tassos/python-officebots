#!/usr/bin/env python3

import logging
logging.basicConfig(level=logging.INFO)

import sys
import tty
import termios

import asyncio

from officebots import Robot

logging.getLogger('officebots').setLevel(logging.DEBUG)

if len(sys.argv) < 2:
    print("Usage: %s <robot name>" % sys.argv[0])
    sys.exit(1)

name = sys.argv[1]


ARROW_UP = "up"
ARROW_DOWN = "down"
ARROW_RIGHT = "right"
ARROW_LEFT = "left"


def get_keypress():
    """
    Non-blocking polling of the keyboard.
    Read max 1 chars from look ahead.
    Process as well common ANSI escape sequences for arrows.
    """
    key = sys.stdin.read(1)

    if key:
        if ord(key) == 27: # ESC!
            sys.stdin.read(1) # we expect a [ here (ANSI CSI sequence)
            ansicode = sys.stdin.read(1)

            if ansicode == "A":
                return ARROW_UP
            elif ansicode == "B":
                return ARROW_DOWN
            elif ansicode == "C":
                return ARROW_RIGHT
            elif ansicode == "D":
                return ARROW_LEFT
            else: # return ESC
                return key

        return key



class MyRobot(Robot):


    async def run(self):

        v = old_v = 0.
        w = old_w = 0.

    
        while True:

            key = get_keypress()

            if key:
                if key == ARROW_UP:
                    v += 0.1
                elif key == ARROW_DOWN:
                    v -= 0.1
                elif key == ARROW_LEFT:
                    w += 0.1
                elif key == ARROW_RIGHT:
                    w -= 0.1
                elif ord(key) == 27:
                    print("Exiting")
                    self.stop()
                    break

                if (v != old_v) or (w != old_w):
                    await self.execute([name, "cmd-vel", [v, w]])
                    old_v = v
                    old_w = w

            await asyncio.sleep(0.1)


orig_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin)

MyRobot().start()

termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
