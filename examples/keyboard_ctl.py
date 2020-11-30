#!/usr/bin/env python3

import logging
logging.basicConfig(level=logging.INFO)

import sys
import termios, fcntl, os

import asyncio
from officebots import Robot

logging.getLogger('officebots').setLevel(logging.WARNING)

if len(sys.argv) < 2:
    print("Usage: %s <robot name>" % sys.argv[0])
    sys.exit(1)

name = sys.argv[1]

attrs_save = None
flags_save = None
fd = sys.stdin.fileno()

ARROW_UP = "up"
ARROW_DOWN = "down"
ARROW_RIGHT = "right"
ARROW_LEFT = "left"
KEY_ESC = chr(27)

def configure_keyboard():
    """
    set keyboard to read single chars lookahead only
    """
    global attrs_save, flags_save

    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save) # copy the stored version to update
    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                  | termios.ISTRIP | termios.INLCR | termios. IGNCR
                  | termios.ICRNL | termios.IXON )
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios. PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)

def restore_keyboard():
    """
    restore previous keyboard settings
    """
    termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)

def get_keypress():
    """
    Non-blocking polling of the keyboard.
    Read max 1 chars from look ahead.
    Process as well common ANSI escape sequences for arrows.
    """
    ret = []

    ret.append(sys.stdin.read(1)) # returns a single character
    fcntl.fcntl(fd, fcntl.F_SETFL, flags_save | os.O_NONBLOCK)
    c = sys.stdin.read(1) # returns a single character
    while len(c) > 0:
        ret.append(c)
        c = sys.stdin.read(1)

    if ret[0]:
        if ord(ret[0]) == 3: # Ctrl+C
            raise KeyboardInterrupt()

        if ret[0] == KEY_ESC: # ESC!
            if len(ret) == 3:
                ansicode = ret[2] 

                if ansicode == "A":
                    return ARROW_UP
                elif ansicode == "B":
                    return ARROW_DOWN
                elif ansicode == "C":
                    return ARROW_RIGHT
                elif ansicode == "D":
                    return ARROW_LEFT
                else: # return ESC
                    return ret[0]

        return ret[0]



class MyRobot(Robot):


    async def on_robot_update(self, data):
        #print(data)
        pass

    async def run(self):

        await self.execute([name, "create"])

        v = old_v = 0.
        w = old_w = 0.

    
        while True:

            key = get_keypress()

            if key:
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

            await asyncio.sleep(0.01)

configure_keyboard()

MyRobot().start()

restore_keyboard()
