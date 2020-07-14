################# Instructions to Run the code ##################
Make sure you have python 2 installed with following libraries.
       scipy
       numpy

 Run the python file
      Example: In linux, open a terminal and type "python course5_assignment1.py"


Notes on Test Scenario
======================

The object ABCD has four contacts. The X,Y coordinate frame is shown by the red arrows.

Two contacts at point A (With contact normals u and v)
Another two contacts at point D (With contact normals a and w)

The relevant contacts for each contact normal in the format of (x,y,theta):
i.e. (x,y) contact point and theta contact angle

u : [3,5,4.712]
v : [3,5,0.785]

a : [12,5,4.712]
w : [12,5,2.356]

TestCase 1 (Form Closure example)
=================================
All 4 contacts are present

Program input: [[3,5,4.712],[3,5,0.785],[12,5,4.712],[12,5,2.356]]

TestCase 2 (Non Form Closure example)
=====================================
Third contact (a) is removed

Program input: [[3,5,4.712],[3,5,0.785],[12,5,2.356]]
