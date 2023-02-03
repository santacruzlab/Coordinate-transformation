'''

Rotation using Rodrigue's Rotation Formula

Rodrigues' rotation calculates the vector that rotates about an arbitrary angle by right-hand rule.
Thus, the rotation of the micromanipulator arm can be streamline into two steps.

1. Rotating the azimuthal angle (phi):
    1) The vector to be rotated is the cartesian coordinate of the tip wrt the pivot point.
    2) The axis is positive z axis
    
2. Rotating the polar angle (theta):
    1) The vector is already rotated once and will be rotated again.
    2) The axis for rotating polar angle is [-1,0,0] but has to rotate phi first.

'''

import numpy as np

pi, sin, cos, atan = np.pi, np.sin, np.cos, np.arctan2

toDegree = lambda rad: rad*180/pi
toRadian = lambda ang: ang*pi/180


def reference(diff,atlas):
    '''
    The difference between the stereotax orthogonal coordinate and the atlas coordinate is calculated based on a reference point.
    The reference point can be the center of the skull, a skull screw, or even a point inside of a chamber.
    The stereotax coordinate is straightforward, but we will need to guesstimate what that point is in the atlas.
    This procedure can render some errors, but there's no way out.
    The difference is calculated as stereo-atlas collected from the reference point.

    INPUT
    diff, atlas: list of three items.
    '''

    stereo = np.array(atlas) + np.array(diff)
    print(f'Atlas {atlas} is sterotax {list(stereo)}.')

    return stereo


def determine_angle(start,target):
    '''
    This basic function assumes the entire micromanipulator arm + syringe pump is a point.
    Rotating in both axes does not result in displacement in AP or ML.

    INPUT
    start:  list of three items. The coordinate of the starting point.
    target: list of three items. The coordinate of the target point.

    OUTPUT
    theta, phi: int. The rotation angles.
    distance: int. The traveling distance to reach the target point.
    '''

    start,target = np.array(start),np.array(target)
    d = target-start
    dx,dy,dz = d[0],d[1],d[2]

    ## Description to prevent wrong directions
    change_x = 'anterior' if dx > 0 else 'posterior'
    change_y = 'lateral'  if dy > 0 else 'medial'
    if not dx: change_x = '0 in x'
    if not dy: change_y = '0 in y'

    # The directionality of micromanipulator arm and the atlas is the opposite.
    # See README for details.
    dy *= -1

    if not dz:
        return 'Not moving downward'  # dz cannot be 0, otherwise it's moving laterally
    dxy = np.sqrt(dx**2 + dy**2)
    r   = np.sqrt(dx**2 + dy**2 + dz**2)
    theta = round(toDegree(atan(dz,dxy)) %90, 2)
    phi   = round(toDegree(atan(dy,dx))  %90, 2) if dxy else 0.0 # dxy can be zero -> moving perpendicularly

    print(f'Moving {change_x} and {change_y}.')
    print(f'Rotate polar {theta} degrees.')
    print(f'Rotate azimuthal {phi} degrees.')
    print(f'Distance {r:.2f} mm.')

    return theta,phi


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


def azimuthal(angle=0,length=0):
    '''
    The scale of the azimuthal angles on the micromanipulator arm is in INCH instead of degree.
    One big gap on the scale is 0.1 inch. The diameter of the circle is 1.3 inch.
    This function is to transform from one to the other.

    INPUT
    angle: int, the angle in degree that we need to rotate about the z axis.

    OUTPUT
    length: int, the length indicated on the scale in inches.

    NOTE
    The input and output of this function are exchangeable.
    We can also input the length and output the angle.
    '''

    D = 1.3 # The diameter of the circle

    if angle:
        length = D * pi * angle / 360
    elif length:
        angle = length * 360 / D / pi

    print(f'Angle {angle:.2f} corresponds to {length:.2f} inch or {length*10:.1f} big gaps.')


def rodrigue(vector,axis,angle):
    '''
    Applying Rodrigues' rotation formula.
    https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
    
    INPUT
    vector, axis: list of three items.
    angle: int, in degrees.
    
    OUTPUT
    v_rot: list of three items, 
    '''
    
    angle = toRadian(angle)
    
    ## Define the cross-product matrix
    K = np.array([[0,-axis[2],axis[1]],
                  [axis[2],0,-axis[0]],
                  [-axis[1],axis[0],0]])
    
    ## Define the rotation matrix about k
    I = np.eye(3)
    R = I + sin(angle) * K + (1-cos(angle))* K @ K
    
    ## Rotate
    v_rot = R @ vector
    
    return v_rot


class stereotax:
    
    dx, dy, dz = -25.4, 123.8, -105

    def __init__(self,AP,ML,DV):
        
        ## Original stereotax coordinate
        self.coordinate = [AP,ML,DV]
        
        ## The cartesian coordinate of the starting position
        self.x = stereotax.dx
        self.y = stereotax.dy - ML
        self.z = stereotax.dz + DV
        
        ## The micromanipulator arm basis
        self.ex = [1,0,0]
        self.ey = [0,1,0]
        self.ez = [0,0,1]
    
        ## Store the starting point
        self.start = self.cartesian
    
    
    @property
    def cartesian(self):
        return round(self.x,2), round(self.y,2), round(self.z,2)

    @property
    def basis(self):
        return np.array([self.ex,self.ey,self.ez]).T
    

    def rotate(self, theta, phi):
        
        ## First rotating about azimuthal axis
        self.x, self.y, self.z = rodrigue(self.cartesian, [0,0,1], phi)
        
        ## Also need to rotate for the new axis
        new_axis = rodrigue([-1,0,0], [0,0,1], phi)
        
        ## Next, rotate about polar axis
        self.x, self.y, self.z = rodrigue(self.cartesian, new_axis, theta)
        
        ## Rotate the basis
        self.ey  = rodrigue(self.ey,  [0,0,1], phi)
        self.ey  = rodrigue(self.ey, new_axis, theta)
        self.ez  = rodrigue(self.ez, new_axis, theta)
        
        ## Store the ending point
        self.new = self.cartesian
        
        
    def moveback(self):
            
        inv_basis = np.linalg.inv(self.basis)
        displace  = inv_basis @ (np.array(self.new) - np.array(self.start))
        
        new_x = self.coordinate[0] + displace[0]
        new_y = self.coordinate[1] - displace[1]
        new_z = self.coordinate[2] + displace[2]
        
        self.coordinate = [round(new_x,2), round(new_y,2), round(new_z,2)]
        print(f'Move to {self.coordinate}')
        
        
#%%

diff = [28,36,34.5]
start_atlas  = [20,24,24]
target_atlas = [19,17,12]

print('Starting point')
start = reference(diff,start_atlas)

print('Target point')
end   = reference(diff,target_atlas)

print()
theta, phi = determine_angle(start=start,target=end)
print()


S = stereotax(*start)

print('Cartesian coordinate before rotation\t', S.cartesian)
S.rotate(theta,phi)
print('Cartesian coordinate after rotation\t', S.cartesian)

print()
S.moveback()