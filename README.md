# RVIZ Publisher
![Python Version](rviz_publisher/badges/python-2.7.6-blue.svg)
[![GitHub commit activity the past week, 4 weeks, yea](https://img.shields.io/github/commit-activity/4w/ipa-flg-ma/rviz_publisher.svg)](https://github.com/ipa-flg-ma/rviz_publisher)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/ipa-flg-ma/rviz_publisher.svg)](https://github.com/ipa-flg-ma/rviz_publisher)

Python programm for auto-localisation and setting goal.

## Terminal
The follwing commandline arguments can be passed to `rviz_publisher.py`:

| Short |   Long   | Value | Description |
|:-----|:--------|:-----|:-----------|
| -h | --help | [FLAG] |show this help message and exit |
| -g GOAL | --goal GOAL [GOAL...] | GOAL [float] |goal for robot as x, y, R, P, Y |
| launch | launch | [str] |`.launch`-file to read 'initial_config' from |


## History
**V 1.3.2**
- add `__del__` func with green output

**V 1.3.0**
- `application.py standalone` and `catkin_make atf_atf_nav_test` test:
  - [x] test passed
  - [ ] test failed

**V 1.1.0:**
- added terminal support for `goal` and `launch`-file
- `goal` accepts `x, y, R, P, Y` input

**V 1.0.0:**
- first try
