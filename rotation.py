import numpy as np

pi, sin, cos, atan = np.pi, np.sin, np.cos, np.arctan2

toDegree  = lambda rad: rad*180/pi
toRadian = lambda ang: ang*pi/180


def reference(diff,atlas=[]):
    '''
    The difference between the stereotax orthogonal coordinate and the atlas coordinate is calculated based on a reference point.
    The reference point can be the center of the skull, a skull screw, or even a point inside of a chamber.
    The stereotax coordinate is straightforward, but we will need to guesstimate what that point is in the atlas.
    This procedure can render some errors, but there's no way out.
    The difference is calculated as stereo-atlas collected from the reference point.

    INPUT
    diff,atlas,stereo: list ohhgf three items.
    '''

    if atlas:
        stereo = np.array(atlas) + np.array(diff)
    # elif stereo:
    #     atlas = np.array(stereo) - np.array(diff)

    print(f'Atlas {list(atlas)} is sterotax {list(stereo)}.')

    return stereo


def rotation_main(start, target):
    '''
    This basic function assumes the entire micromanipulator arm + syringe pump is a point.
    Rotating in both axes does not result in displacement in AP or ML.

    INPUT
    start:  list of three items. The atlas coordinate (AP, ML, DV) of the starting point.
    target: list of three items. The atlas coordinate of the target point.

    OUTPUT
    theta, phi: int. The rotation angles of AP and ML planes.
    distance: int. The traveling distance to reach the target point.
    '''

    start,target = np.array(start),np.array(target)
    d = target-start
    dx,dy,dz = d[0],d[1],d[2]

    # The directionality of micromanipulator arm and the atlas is the opposite.
    # See README for details.
    dy *= -1

    if not dz:
        return 'Not moving downward'
    dxy = np.sqrt(dx**2 + dy**2)
    r = round(np.sqrt(dx**2 + dy**2 + dz**2), 2)
    # dz cannot be 0, otherwise it's moving laterally
    theta  = round(atan(dz,dxy) *180/np.pi %90, 2)
    # dxy can be zero -> moving perpendicularly
    phi  = round(atan(dy,dx) *180/np.pi %90, 2) if dxy else 0.0

    print(f'Rotate polar {theta} degrees.')
    print(f'Rotate azimuthal {phi} degrees.')
    print(f'Distance {r} mm.')


def rotateZAxis(angle=0,length=0):
    '''
    The scale of the z axis on the micromanipulator arm is in INCH instead of degree. One big gap on the scale is 0.1 inch. The diameter of the circle is 1.3 inch. This function is to transform from one to the other.

    INPUT
    angle: int, the angle in degree that we need to rotate about the z axis.

    OUTPUT
    TODO: change to gaps.
    length: int, the length indicated on the scale in inches.

    NOTE
    The input and output of this function are exchangeable. We can also input the length and output the angle.
    '''

    D = 1.3 # The diameter of the circle

    if angle:
        length = D * pi * angle / 360
    elif length:
        angle = length * 360 / D / pi

    print(f'Angle {angle:.2f} corresponds to {length*10:.1f} big gaps.')


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
    return r, toDegree(theta), toDegree(phi)


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


    def moveback(self):
        '''
        This function outputs how much to move in each direction in order to reach the strating point.
        '''
        inv_basis = np.linalg.inv(self.basis)
        moving = inv_basis @ (np.array(self.cartesian)-np.array(self.before))

        new_x = self.original[0]-moving[0]
        new_y = self.original[1]+moving[1]
        new_z = self.original[2]-moving[2]

        if -100<new_x<100 and 0<new_y<80 and 0<new_z<78:
            print(f'Move x (AP) to {new_x:.2f}')
            print(f'Move y (ML) to {new_y:.2f}')
            print(f'Move z (DV) to {new_z:.2f}')
        else:
            print(f'New coordinate: [{new_x:.2f}, {new_y:.2f}, {new_z:.2f}]')
            print('Unreachable point.')


putamen_atlas = [16,19,17]
start_atlas = [16,25,27]
diff = [27,36,50]
a = reference(diff,atlas=putamen_atlas)
b = reference(diff,atlas=start_atlas)
rotation_main(start=b,target = a)
S = stereotax(*b)
