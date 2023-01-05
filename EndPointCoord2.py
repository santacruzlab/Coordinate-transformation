# -*- coding: utf-8 -*-
"""
This function calculate the coordinates of the end point of the manipulator. 
The next assumptions are made:
1) The manipulater has been displaced from the origin of the coordinate system 
    a total length= "dist_traveled".
2) The input_angles are ordered respect the AP, ML and DV axes, then coordinates of
    the P_end correspond to the AP,ML and DV axes too. 
3) Since it is expected that all the components of P_end are negative numbers, the angle
    respect AP must be acute, the angle respect ML must be obtuse and the angle respect DV
    must be acute. This convention follows the angles representation as in the Stereotax Fig. of Airport
If it is necessary to change the convention stated above, just consider that the 
the coordinates of the End_point are (d*cos(a1), d*cos(a2), d*cos(a3)), where 
a1, a2 and a3 are the angles with respect AP, ML and DV axes respectively and
d is the displacement.

"""

def EndPointCoord2(input_angles, dist_traveled):  #[AP, ML, DV]
	
    import numpy as np
    input_angles=np.array(input_angles)
    
	# Convert degrees to radians
    input_angles = input_angles * np.pi/180
    
    P_end=dist_traveled*[-1*np.cos(input_angles[0]), np.cos(input_angles[1]), -1*np.cos(input_angles[2])]
    
    return P_end


#np.array([74.40, 107.06, 23.45])	# values are in degrees, [AP, ML, DV]
    


 