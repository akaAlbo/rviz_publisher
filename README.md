# RVIZ Publisher
[![Python Version](rviz_publisher/rviz_publisher/badges/python-2.7.6-blue.svg)
Python programm for auto-localisation and setting goal.

## Terminal
The follwing commandline arguments can be passed to `rviz_publisher.py`:

| Short |   Long   | Value | Description |
|:-----|:--------|:-----|:-----------|
| -h | --help | [FLAG] |show this help message and exit |
| -g GOAL | --goal GOAL [GOAL...] | GOAL [float] |goal for robot as x, y, R, P, Y |
| launch | launch | [str] |`.launch`-file to read 'initial_config' from |


## History
**V 1.3.0**
- `application.py standalone` and `catkin_make atf_atf_nav_test` test:
  - [x] test passed
  - [ ] test failed 

**V 1.1.0:**
- added terminal support for `goal` and `launch`-file
- `goal` accepts `x, y, R, P, Y` input

**V 1.0.0:**
- first try
