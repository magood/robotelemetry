import unittest
from gpsd_kalman_loop import *
import numpy as np

class Test_kf_test(unittest.TestCase):
    def test_kf_zeros(self):
        x = np.array([0., 0., 0., 0., 0., 0., 0.])
        #F - Next state update matrix, nxn
        F = np.matrix([[1.,0.,0.,1.,0.,0.,0.],
                       [0.,1.,0.,0.,1.,0.,0.],
                       [0.,0.,1.,0.,0.,0.,0.],
                       [0.,0.,0.,1.,0.,0.,0.],
                       [0.,0.,0.,0.,1.,0.,0.],
                       [0.,0.,0.,0.,0.,1.,0.],
                       [0.,0.,0.,0.,0.,0.,1.]])
        #measurement is like m = [x, y, altitude, dx, dy (computed from speed/track), speed, track]
        #H - Measurement extraction matrix (mxn), maps state back to measurement form...
        H = np.matrix([[1.,0.,0.,0.,0.,0.,0.],
                       [0.,1.,0.,0.,0.,0.,0.],
                       [0.,0.,1.,0.,0.,0.,0.],
                       [0.,0.,0.,1.,0.,0.,0.],
                       [0.,0.,0.,0.,1.,0.,0.],
                       [0.,0.,0.,0.,0.,1.,0.],
                       [0.,0.,0.,0.,0.,0.,1.]])
        #P - covariance matrix of x state, nxn
        P = np.matrix([[1000.,0.,0.,0.,0.,0.,0.],
                       [0.,1000.,0.,0.,0.,0.,0.],
                       [0.,0.,1000.,0.,0.,0.,0.],
                       [0.,0.,0.,1000.,0.,0.,0.],
                       [0.,0.,0.,0.,1000.,0.,0.],
                       [0.,0.,0.,0.,0.,1000.,0.],
                       [0.,0.,0.,0.,0.,0.,1000.]])
        GPS_LOC_VARIANCE = 13.738173188348601860982040259622
        GPS_V_VARIANCE = 0.1 ** 2
        GPS_TRACK_VARIANCE = 1.0
        R = np.matrix([[GPS_LOC_VARIANCE,0.,0.,0.,0.,0.,0.],
                       [0.,GPS_LOC_VARIANCE,0.,0.,0.,0.,0.],
                       [0.,0.,GPS_LOC_VARIANCE,0.,0.,0.,0.],
                       [0.,0.,0.,GPS_V_VARIANCE,0.,0.,0.],
                       [0.,0.,0.,0.,GPS_V_VARIANCE,0.,0.],
                       [0.,0.,0.,0.,0.,GPS_V_VARIANCE,0.],
                       [0.,0.,0.,0.,0.,0.,GPS_TRACK_VARIANCE]])
        u = np.array([0.,0.,0.,0.,0.,0.,0.])
        I = np.matrix([[1.,0.,0.,0.,0.,0.,0.],
                    [0.,1.,0.,0.,0.,0.,0.],
                    [0.,0.,1.,0.,0.,0.,0.],
                    [0.,0.,0.,1.,0.,0.,0.],
                    [0.,0.,0.,0.,1.,0.,0.],
                    [0.,0.,0.,0.,0.,1.,0.],
                    [0.,0.,0.,0.,0.,0.,1.],])
        z = np.array([0, 0, 0, 0, 0, 0, 0])
        x,P = kalman_step(x, F, u, P, z, H, R, I)
        np.testing.assert_array_equal(x, np.array([0., 0., 0., 0., 0., 0., 0.]))

    def test_kf_simiple(self):
        x = np.array([0., 0., 0., 0., 0., 0., 0.])
        #F - Next state update matrix, nxn
        F = np.matrix([[1.,0.,0.,1.,0.,0.,0.],
                       [0.,1.,0.,0.,1.,0.,0.],
                       [0.,0.,1.,0.,0.,0.,0.],
                       [0.,0.,0.,1.,0.,0.,0.],
                       [0.,0.,0.,0.,1.,0.,0.],
                       [0.,0.,0.,0.,0.,1.,0.],
                       [0.,0.,0.,0.,0.,0.,1.]])
        #measurement is like m = [x, y, altitude, dx, dy (computed from speed/track), speed, track]
        #H - Measurement extraction matrix (mxn), maps state back to measurement form...
        H = np.matrix([[1.,0.,0.,0.,0.,0.,0.],
                       [0.,1.,0.,0.,0.,0.,0.],
                       [0.,0.,1.,0.,0.,0.,0.],
                       [0.,0.,0.,1.,0.,0.,0.],
                       [0.,0.,0.,0.,1.,0.,0.],
                       [0.,0.,0.,0.,0.,1.,0.],
                       [0.,0.,0.,0.,0.,0.,1.]])
        #P - covariance matrix of x state, nxn
        P = np.matrix([[1000.,0.,0.,0.,0.,0.,0.],
                       [0.,1000.,0.,0.,0.,0.,0.],
                       [0.,0.,1000.,0.,0.,0.,0.],
                       [0.,0.,0.,1000.,0.,0.,0.],
                       [0.,0.,0.,0.,1000.,0.,0.],
                       [0.,0.,0.,0.,0.,1000.,0.],
                       [0.,0.,0.,0.,0.,0.,1000.]])
        GPS_LOC_VARIANCE = 13.738173188348601860982040259622
        GPS_V_VARIANCE = 0.1 ** 2
        GPS_TRACK_VARIANCE = 1.0
        R = np.matrix([[GPS_LOC_VARIANCE,0.,0.,0.,0.,0.,0.],
                       [0.,GPS_LOC_VARIANCE,0.,0.,0.,0.,0.],
                       [0.,0.,GPS_LOC_VARIANCE,0.,0.,0.,0.],
                       [0.,0.,0.,GPS_V_VARIANCE,0.,0.,0.],
                       [0.,0.,0.,0.,GPS_V_VARIANCE,0.,0.],
                       [0.,0.,0.,0.,0.,GPS_V_VARIANCE,0.],
                       [0.,0.,0.,0.,0.,0.,GPS_TRACK_VARIANCE]])
        u = np.array([0.,0.,0.,0.,0.,0.,0.])
        I = np.matrix([[1.,0.,0.,0.,0.,0.,0.],
                    [0.,1.,0.,0.,0.,0.,0.],
                    [0.,0.,1.,0.,0.,0.,0.],
                    [0.,0.,0.,1.,0.,0.,0.],
                    [0.,0.,0.,0.,1.,0.,0.],
                    [0.,0.,0.,0.,0.,1.,0.],
                    [0.,0.,0.,0.,0.,0.,1.],])
        zs = []
        zs.append(np.array([0, 0, 0, 0, 0, 0, 0]))
        zs.append(np.array([1, 1, 0, 0, 0, 0, 0]))
        zs.append(np.array([-1, 1, 0, 0, 0, 0, 0]))
        zs.append(np.array([-1, -1, 0, 0, 0, 0, 0]))
        zs.append(np.array([1, -1, 0, 0, 0, 0, 0]))
        for z in zs:
            x,P = kalman_step(x, F, u, P, z, H, R, I)
        np.testing.assert_allclose(x, np.array([0., 0., 0., 0., 0., 0., 0.]))

if __name__ == '__main__':
    unittest.main()