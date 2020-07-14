Make sure you have python 2 installed with following libraries.
       Numpy
       CSV Library


To run the code, go to the /code directory and simply run the course4_assignment2.py file.
  Linux Example: 
           Open a terminal and go to "code" folder
           Execute "python course4_assignment2.py"

NOTES ON EXECUTION
==================
Please make sure to have the parameters.csv file and the obstacles.csv file within the /code directory, before running the code. 

The the parameters.csv files are included in the results folders. 
The structure of the parameter file is described in the file itself. 

The code will save 3 files.
	nodes.csv
	edges.csv
	path.csv

The strcuture of these files are of the format used by Scene2_UR5_csv scene. 
For additional information, please refer the generated files. 

NOTES ON IMPLEMENTATION
=======================
The code uses a PRM planner, since a PRM planner is resolution complete, for a given sampling resolution of the environment. 

The user can adjust the performance of the planner, using the parameters.csv file. 
The main performance parameters are:
	# Sample Resolution
		Given a sample resolution value of "x", space will be sampled at distances of "x"
	# Proximity Threshold
		Given a proximity threshold of "y", only nodes within a radius of "y" distance from a node,
		will be considered as neighbours of the node. 
		In other words the cost of any edge in the PRM graph is less than "y"
        # Robot Diameter
		The obstacle radius values are inflated by the robot diameter, before finding the path.
		A default value of 12cm robot diameter is used in the generated results. 
		However, user can change that value if needed.

NOTES ON COLISION DETECTION
===========================
To determine whether the path between two points is in collision with an obstacle, the following approach is used.

The line from point A (x1, y1) to point B (x2, y2) is described in using the parameter t as follows

x = x1 + t(x2 - x1)
y = y1 + t(y2 - y1)

0 <= t <= 1

Assume the above path collides with the circular obstacle at coordinates (a,b) with radius r

The equation of the obstacle circle is,

 (x - a)^2 + (y - b)^2 = r^2

By applying the parametric eqation of the line, in the equation of the circle and solving,

if any solution exists -> The line intersects the obstacle
if 0 <= t <= 1 for the solution -> The intersection is between point A and B. Hence the path is in collision.

