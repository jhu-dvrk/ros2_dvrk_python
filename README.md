# dVRK Python client library

This package is based on the CRTK Python client library: crtk-robotics.readthedocs.io

# Example of use

For the dVRK, one can use the classes `dvrk.arm`, `dvrk.psm`,
`dvrk.mtm`... that use the `crtk.utils` to provide as many features as
possible.  This is convenient for general purpose testing, for example
in combination with iPython to test snippets of code.

:warning: There is a significant performance penalty when using the
`dvrk.xxx` classes since they subscribe to more topics than generally
needed.  For your application, it is recommended to use your own class
and only add the features you need to reduce the number of ROS
messages and callbacks.  See examples in the directory `scripts`,
e.g. `dvrk-bag-replay.py`.

Example using `dvrk.arm`:

```python
import dvrk
p = dvrk.arm('PSM1')
p.enable()
p.home()

# get measured joint state
[position, velocity, effort, time] = p.measured_js()
# get only position
position = p.measured_jp()
# get position and time
[position, time] = p.measured_jp(extra = True)

# move in joint space
import numpy
p.move_jp(numpy.array([0.0, 0.0, 0.10, 0.0, 0.0, 0.0]))

# move in cartesian space
import PyKDL
# start position
goal = p.setpoint_cp()
# move 5cm in z direction
goal.p[2] += 0.05
p.move_cp(goal).wait()

import math
# start position
goal = p.setpoint_cp()
# rotate tool tip frame by 25 degrees
goal.M.DoRotX(math.pi * 0.25)
p.move_cp(goal).wait()
```
