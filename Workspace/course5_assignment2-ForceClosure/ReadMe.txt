################# Instructions to Run the code ##################
Make sure you have python 2 installed with following libraries.
       scipy
       numpy

 Run the python file
      Example: In linux, open a terminal and type "python course5_assignment1.py"


Notes on Own Design Test Scenario
=================================

The assembly has 3 bodies. 
	ABCD Rectangular body with 10kg mass at (1, 2.5)
	E Circular body with 3kg mass at (4, 3)
	HIKJ Polygonal body with 5kg mass at (8, 3)


The Assembly has contacts at points A, B, F, I, K, P
The contacts normals and external forces (body weights) are shown in the diagram.

Body Descriptions: 

	ABCD body- [1, 1, 2.5, 10]
	E Circle - [2, 4, 3, 3]
	HIKJ body- [3, 8, 3, 5]

Contact Descriptions:
	Point A- [1, 0, 0, 0, 1.5708, u1]
	Point B- [1, 0, 2, 0, 1.5708, u1]
	Point F- [2, 1, 1, 3, 0, u1]
	Point I- [3, 0, 3.83, 0, 1.5708, u2]
	Point K- [3, 0, 10, 0, 1.5708, u2]
	Point P- [2, 3, 5.44, 1.61, 2.356, u2]


NOTE:
	Each body description is in format (ID,x,y,mass):
		BodyID, (x,y) coordinate of body mass center and body mass in Kilograms
		BodyID 0 is stationary ground

	Each contact description is in format (bodyA, bodyB, x, y, theta, u):
        	Contact from bodyB into bodyA
        	(x,y) : Coordinates of contact point
        	theta : Angle of contact normal into bodyA (in Radians)
         	u : Coulomb Friction Coefficient


TestCase 1 (Force Closure example)
=================================
Take u1 = 0.5 and u2 = 0.5

Body Descriptions: [[1, 1, 2.5, 10], [2, 4, 3, 3], [3, 8, 3, 5]]
Contact Descriptions: [[1, 0, 0, 0, 1.5708, 0.5], [1, 0, 2, 0, 1.5708, 0.5], [2, 1, 1, 3, 0, 0.5], [3, 0, 3.83, 0, 1.5708, 0.5], [3, 0, 10, 0, 1.5708, 0.5], [2, 3, 5.44, 1.61, 2.356, 0.5]]


TestCase 2 (Non Form Closure example)
=====================================
Take u1 = 0.5 and u2 = 0.05

Body Descriptions: [[1, 1, 2.5, 10], [2, 4, 3, 3], [3, 8, 3, 5]]
Contact Descriptions: [[1, 0, 0, 0, 1.5708, 0.5], [1, 0, 2, 0, 1.5708, 0.5], [2, 1, 1, 3, 0, 0.5], [3, 0, 3.83, 0, 1.5708, 0.05], [3, 0, 10, 0, 1.5708, 0.05], [2, 3, 5.44, 1.61, 2.356, 0.05]]
