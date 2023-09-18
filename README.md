OfficeBots -- Python API
========================

This package implements the OfficeBots API in Python, making it easy to
create and control a robot in the OfficeBots game.

Installation
------------


### From pypi

```
> pip3 install officebots
```


### From sources

```
> python3 setup.py install --user
```


Usage
-----

Check examples in the `examples/` directory. You can start with the simple
`cmdline.py`.

### List of available commands

These commands can all be used with the `cmdline.py` script.
For instance: `python3 examples/cmdline.py set-pos 1.3 2.3 0.5` will position
the robot to coordinate (1.3m, 2.3m) and orientation 0.5 rad.

- `cmd-vel <linear velocity> <angular velocity>`: sets the velocity of the robot
- `set-pos <x> <y> <theta>`: sets the position of the robot (in meters and
  radians)
- `navigate-to <x> <y>`: navigate to the provided point (note that the final
  orientation can not be controlled)
- `stop`: stops the robot if moving
- `say <message>`
- `get-pos`: returns the current position of the robot
- `set-color <colour>`: sets the colour of the robot. Parameter must be one of
  `black`, `blue`, `yellow`, `green`, `red`, `white`, `purple`, `beige`.
- `set-screen <base64-encoded JPG>`: set the image on the robot's screen. See
  [the source of cmdline.py](examples/cmdline.py) for an example of how to
  create the base64 JPG image.
- `get_humans`: returns the humans currently visible to the robot. For each of
  them, returns the name, (`x`, `y`, `theta`) position and the 6D head position
  (useful to compute eg the gaze direction)


ROS support
-----------

This Python package also provides a complete ROS bridge to the simulator,
supporting:

- `/cmd_vel` to move the robot
- laser scan simulation (`/scan`)
- tf frames
- ROS4HRI implementation (with persons and faces tracking, and speech simulation
  via the in-game chat)
- simulated navigation (via `/move_base_simple/goal`)
- image streaming to the robot's tablet (by publishing images to
  `/screen/raw_image`)
- simulated robot speech (`/say`)
