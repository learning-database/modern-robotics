import numpy as np
import math



H0 = np.array([ [-2.828, 0.707, -0.707], 
                [-2.828, 0.707, 0.707],
                [-2.828, -0.707, 0.707],
                [-2.828, -0.707, -0.707]])

H0 = H0 * 4

#Vb = np.array([1, 0, 0])
#Vb = np.array([0, 0, 3.536])
Vb = np.array([1,2,3])
print np.dot(H0, Vb)