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
