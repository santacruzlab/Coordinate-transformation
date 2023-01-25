import numpy as np

pi, sin, cos, atan = np.pi, np.sin, np.cos, np.arctan2

toAngle  = lambda rad: rad*180/pi
toRadian = lambda ang: ang*pi/180


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
    
## Example
S = stereotax(-16,20.4,39)
S.rotate(5,10)
print(S.move())
