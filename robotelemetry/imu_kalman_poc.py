import numpy as np
from math import radians, degrees, sin, cos, sqrt, asin, atan2, pi

if __name__ == '__main__':
    #local plane is tangent to earth at 0,0.  Positive y is North.
    #x, u, and z should be column-vectors
    #x has n elements.
    n = 12
    dt = 1.
    
    #x = [x, y, z, yaw/heading, pitch/inclination, roll/tilt, ax, ay, az, gx, gy, gz]
    x = np.matrix([[0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]]).T

    #F - Next state update matrix, nxn
    F = np.identity(n)
    F[0,6] = 1.0 #new x is x + ax
    F[1,7] = 1.0
    F[2,8] = 1.0
    F[3,11] = 1.0 #new yaw is yaw + gz. Pretty sure gz b/c it is rotation about the Z axis.
    F[4,9] = 1.0 #new pitch is pitch + gx, rotation about x
    F[5,10] = 1.0 #new roll is roll + gy

    #Z from IMU: [ax,ay,az,gx,gy,gz,mx,my,mz]
    #Magnetometer measures the magnetic flux density in uT or microteslas?
    #Magnetometer values describe a vector pointing to "magnetic north" more or less.
    #I think what I need to do is just project this onto the 2d plane, which is trivial to do, right?
    #In this case, it's just mx and my??
    #Then find the angle this vector describes and that's heading, relative to something, maybe not my coordinate system.
    #Maybe I should change that...
    m=9

    #H - Measurement extraction matrix (mxn), maps state back to measurement form...
    #Hx = m
    H = np.matrix([[0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.,0.], #this row * state ax = measured ax
                   [0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.,0.], #ay
                   [0.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.,0.], #az
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,0.,0.], #gx
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.,0.], #gy
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,1.], #gz
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.], #mx? (I'm not actually sure how these magnetometer readings work)
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.], #my?
                   [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]]) #mz?