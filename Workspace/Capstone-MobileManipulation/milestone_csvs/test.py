from __future__ import division
import modern_robotics as mr
import numpy as np
import math
import sys

Xd =      [[0.018, 0.0, 1.0, 0.587], [0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.018, 0.463], [0.0, 0.0, 0.0, 1.0]]
Xd_next = [[0.018, 0.0, 1.0, 0.795], [0.0, 1.0, 0.0, 0.0], [-1.0, 0.0, 0.018, 0.463], [0.0, 0.0, 0.0, 1.0]]
print "hello"
print "hello"
print "hello"
Xt = np.dot( np.linalg.inv(Xd), Xd_next)
for x in range(5):
    sys.stdout.write("\033[F")
    print 'Generating youBot trajectory controls. Progress'
    