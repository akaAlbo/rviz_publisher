# RVIZ Publisher
Python programm for auto-localisation and setting goal.

## Terminal
The follwing commandline arguments can be passed to `rviz_publisher.py`:

| Short |   Long   | Value | Description |
|:-----|:--------|:-----|:-----------|
| -h | --help | [FLAG] |show this help message and exit |
| -g GOAL | --goal GOAL [GOAL...] | GOAL [float] |goal for robot as x, y, R, P, Y |
| launch | launch | [str] |`.launch`-file to read 'initial_config' from |


## History
**V 1.1.0:**
- added terminal support for `goal` and `launch`-file
- `goal` accepts `x, y, R, P, Y` input

**V 1.0.0:**
- first try
