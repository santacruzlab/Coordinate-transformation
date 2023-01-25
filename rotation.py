'''

This script calculates the transformation between the stereotax coordinates and the atlas coordinates.



'''

import numpy as np


def rotation_main(start, target):
    '''
    
    This basic function assumes the entire micromanipulator arm + syringe pump is a point.
    Rotating in both axes does not result in displacement in AP or ML.
    
    INPUT
    start:  list of three items. The atlas coordinate (AP, ML, DV) of the starting point.
    target: list of three items. The atlas coordinate of the target point.
    
    OUTPUT
    rotation: list of two items. The rotation angles of AP and ML planes.
    distance: int. The traveling distance to reach the target point.
    '''
    
    start,target = np.array(start),np.array(target)
    d = target-start
    dx,dy,dz = d[0],d[1],d[2]
    if not dz:
        return 'Not moving downward'
    dxy = np.sqrt(dx**2 + dy**2)
    distance = round(np.sqrt(dx**2 + dy**2 + dz**2), 2)
    # dz cannot be 0, otherwise it's moving laterally
    theta_1  = round(np.arccos(dz/distance) *180/np.pi %90, 2)
    # dxy can be zero -> moving perpendicularly
    theta_2  = round(np.arccos(dx/dxy) *180/np.pi %90, 2) if dxy else 0.0
    
    return theta_1, theta_2, distance

def rotateZAxis(angle=0,length=0):
    '''
    The scale of the z axis on the micromanipulator arm is in INCH instead of degree. One big gap on the scale is 0.1 inch. The diameter of the circle is 1.3 inch. This function is to transform from one to the other.
    
    INPUT
    angle: int, the angle in degree that we need to rotate about the z axis.
    
    OUTPUT
    length: int, the length indicated on the scale in inches.
    
    NOTE
    The input and output of this function are exchangeable. We can also input the length and output the angle.
    '''
    
    D = 1.3 # The diameter of the circle
    
    if angle:
        return D*pi/360 * angle
    elif length:
        return length * 360/D/pi


def spherical_cartesian(r,theta,phi):
    '''
    Tranform from spherical to cartesian coordinates.
    theta and phi are in angles
    '''
    theta = toRadian(theta)
    phi = toRadian(phi)
    
    x = r * cos(theta) * cos(phi)
    y = r * cos(theta) * sin(phi)
    z = r * sin(theta)
    
    return x,y,z
    



#%%

pi, sin, cos, atan = np.pi, np.sin, np.cos, np.arctan2

toAngle  = lambda rad: rad*180/pi
toRadian = lambda ang: ang*pi/180

def cartesian_spherical(x,y,z):
    '''
    Tranform from cartesian to spherical coordinates.
    theta and phi are in angles
    '''
    r   = np.sqrt( x**2 + y**2 + z**2 )
    dxy = np.sqrt( x**2 + y**2 )
    theta = atan(z,dxy) # angle with the z=0 plane
    phi   = atan(y,x)  # angle on the xy plane
    return r, toAngle(theta), toAngle(phi)


class stereotax:
    
    def __init__(self,AP,ML,DV):
        
        '''
        Adjust the xyz distance so that when we plug in the read number on the stereotax, 
        it shows the actually coordinate relative to the pivot point.
        '''
        self.original = [AP,ML,DV]
        self.x = 25.4
        self.y = 123.8-ML
        self.z = DV-75
        
        ## The micromanipulator arm basis
        self.ex = [1,0,0]
        self.ey = [0,1,0]
        self.ez = [0,0,1]
        
        self.r, self.theta, self.phi = cartesian_spherical(self.x, self.y, self.z)
        self.before = self.cartesian
        
    @property
    def cartesian(self):
        return round(self.x,2), round(self.y,2), round(self.z,2)

    
    @property
    def spherical(self):
        '''theta and phi are in angles'''
        return round(self.r,2), round(self.theta,2), round(self.phi,2)
        
    
    def rotate(self,theta,phi):
        '''
        Theta is the angle with the z=0 plane.
        Phi is the angle with y=0 ON the xy plane.
        '''
        
        self.theta += theta
        self.phi += phi

        t = toRadian(self.theta)
        p = toRadian(self.phi)
        
        self.x = self.r * cos(t) * cos(p)
        self.y = self.r * cos(t) * sin(p)
        self.z = self.r * sin(t)
        
        ## New basis
        self.ex = [1,0,0]
        self.ey = spherical_cartesian(1, 90+theta, 0+phi)
        self.ez = spherical_cartesian(1, 0+theta, 90+phi)
        self.basis = np.array([self.ex,self.ey,self.ez]).T
        

    def move(self):
        
        inv_basis = np.linalg.inv(self.basis)
        moving = inv_basis @ (np.array(self.cartesian)-np.array(self.before))
        
        print(f'Move x (AP) to {self.original[0]-moving[0]}')
        print(f'Move y (ML) to {self.original[1]+moving[1]}')
        print(f'Move z (DV) to {self.original[2]-moving[2]}')
        
        return moving
    
#%%
S = stereotax(-16,20.4,39)
S.rotate(5,10)
print(S.move())

"""
These functions are no longer needed.


def Rx(angle):
    '''
    Return the rotation matrix about the x axis
    '''
    angle /= (180/pi)
    return np.array([[1,0,0],
                   [0,cos(angle),-sin(angle)],
                   [0,sin(angle),cos(angle)]])
 
def Rz(angle):
    '''
    Return the rotation matrix about the z axis
    '''
    angle /= (180/pi)
    return np.array([[cos(angle),-sin(angle),0],
                   [sin(angle), cos(angle),0],
                   [0,0,1]])

def findCoordinate(NP, ML, DV):
    '''
    This function calculates the cartesian coordinates wrt the pivot point, which is the rotating center of the x axis. That is, we know NP, and we have to find NPx, NPy, and NPz (x,y,z). There are several variables defined here. 
    
    1. Needle tip (N): the point we are interested in. This function will output the coordinate of this point wrt the pivot point.
    2. Pivot point (P): the rotating center of the x axis. Note that rotating about the z axis also has the same pivot point here. It's the zero (0,0,0).
    3. Rotating rod (R): The rod of the syringe pump can be rotated. The angle is unknown but that does not matter. If we have the length from N to P (denoted as NP), then we should be able to use trigonometry to find the coordinate without knowing the angle.
    
    The measured variables are collected under these assumptions
    1. Stereotax axes are orthogonal, meaning no rotation is present during calculation.
    2. ML displacement is 80 mm, meaning the 0 points to the 8 on the ML scale.
    3. DV displacement is 75 mm, meaning the 0 points to the 7.5 on the DV scale.
    4. The syringe rod is placed downward and placed to the end. That means the cap of the rotating rod is facing down and there is no gap between the arm adaptor and the bottom of the rod.

    INPUT
    NP: int, measured distance between N and P in mm.
    ML: int, the displacement of ML reading on the micromanipulator arm in mm.
    DV: int, the displacement of DV reading on the micromanipulator arm in mm.  
    
    OUTPUT
    coordinate: list of three items. The (x,y,z) or (AP, ML, DV) coordinates in mm.
    
    TODO
    what if the stereotax is already at an angle?
    '''
    
    # These variables are eyeballed. Not 100% accurate.
    measured_PRy = 51.5
    measured_NPz = 31.2
    measured_NR  = 67.6
    measured_ML_displacement = 80.0
    measured_DV_displacement = 75.0
    
    # These variables are derived.
    theta = sp.symbols('theta')
    NPz = DV - measured_NPz - measured_DV_displacement
    NPy = measured_PRy + measured_ML_displacement - ML + measured_NR * sp.cos(theta)
    NPx = measured_NR * sp.sin(theta)
    
    # Solve the theta using sympy
    sol = sp.solve( NP**2 - NPz**2 - NPy**2 - NPx**2 )
    
    # Apply the theta to NPy and NPx
    theta = float(sol[1])
    NPy = measured_PRy + measured_ML_displacement - ML + measured_NR * cos(theta)
    NPx = measured_NR * sin(theta)
    
    NPx,NPy,NPz = round(NPx,2),round(NPy,2),round(NPz,2)
    
    return theta*180/pi, [NPx,NPy,NPz]
    

def moveAfterRotation(rotate, start):
    '''
    This function calculates how far to move in each axis after rotation. The coordinate based on the new basis is calculated by the inverse of the basis @ the original coordinate.
    
    INPUT
    rotate: numpy array of 3x3, the rotation matrix applied to the manipulator arm.
    start:  list of three items, the starting point (x,y,z) wrt to the pivot point that will be rotated. 
    
    OUPTUT
    move: numpy array of 1x3, the movement corresponding to the new basis.
    '''
    
    # New basis for each axis
    x = [1,0,0]
    y = rotate @ [0,1,0]
    z = rotate @ [0,0,1]
    basis = np.array([x,y,z]).T
    
    # Inverse of the basis
    inv_basis = np.linalg.inv(basis)
    
    # Calculate the end point and the movement required to reset the position.
    end  = rotate @ start
    move = inv_basis @ (end-start)
    
    return move


"""
