import math

d = 1.5

if (math.fabs(d) > 1):
    print "Odometry Jump Detected. Ignoring current data and using last known good odometry data"
    d_left = 0
    d_right = 0
    d = 0
    

if d>1:
    d=0
    th=0
    print "Existing hack executed"

print math.atan2(2,0)

print '{:5.3f}'.format(10)
		   
           