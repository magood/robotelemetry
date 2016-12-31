import gpsd
import time, os
import numpy as np
from math import radians, degrees, sin, cos, sqrt, asin, atan2
from flask_socketio import SocketIO

def connect_local():
    gpsd.connect()

def coord_distance(p1, p2):
    """Estimated distance between two lat/lon points using haversine.
    p1 and p2 should be [lat,lon]."""
    #references: http://www.movable-type.co.uk/scripts/latlong.html
    R = 6372.8 * 1000 #in meters
    rlat1 = radians(p1[0])
    rlat2 = radians(p2[0])
    dLat = radians(p2[0] - p1[0])
    dLon = radians(p2[1] - p1[1])
    a = sin(dLat/2)**2 + cos(rlat1)*cos(rlat2)*sin(dLon/2)**2
    c = 2*asin(sqrt(a))
    dist = R*c
    return dist

def bearing_to_point(p1, p2):
    """initial bearing (in radians) - over long distances, a straight line may change heading."""
    #references: http://www.movable-type.co.uk/scripts/latlong.html
    rlat1 = radians(p1[0])
    rlat2 = radians(p2[0])
    dLat = radians(p2[0] - p1[0])
    dLon = radians(p2[1] - p1[1])
    y1 = sin(dLon) * cos(rlat2);
    x1 = cos(rlat1)*sin(rlat2) - sin(rlat1)*cos(rlat2)*cos(dLon);
    brng = atan2(y1, x1); #from origin, in radians
    return brng

def bearing_dist_to_plane(bearing, distance):
    """x,y coordinates for a point distance units from 0,0 at angle bearing"""
    x = Cos(bearing) * distance
    y = Sin(bearing) * distance
    return [x,y]

def coords_to_xy(lat, lon):
    """Converts latitude/longitude coordinates to local x/y plane, using Haversine"""
    #references: http://gamedev.stackexchange.com/questions/18340/get-position-of-point-on-circumference-of-circle-given-an-angle
    R = 6372.8 * 1000 #in meters
    if ORIGIN_LOC:
        point = [lat,lon]
        dist = coord_distance(ORIGIN_LOC, point)
        #Now, if I knew what direction...
        brng = bearing_to_point(ORIGIN_LOC, point)
        #to x/y
        return bearing_dist_to_plane(brng, dist)

def set_origin():
    """Wait for fix, set origin in local plane as first point with at least a 2D fix"""
    FIX_MODE = 1
    ORIGIN_LOC = None

    while FIX_MODE < 2:
        p = gpsd.get_current()
        FIX_MODE = p.mode
        if p.mode >= 2:
            ORIGIN_LOC = (p.lat, p.lon)
        time.sleep(1)

def hack_time():
    """dumb dirty hack to set the time from the gps"""
    starttime = None
    while starttime is None:
        p = gpsd.get_current()
        starttime = p.time
    newtime = starttime
    while newtime == starttime:
        p = gpsd.get_current()
        if p.mode >= 2:
            newtime = p.time
    #set the system time.
    #os.system('date -s %s' % newtime)
    os.system('hwclock --set %s' % newtime)

def reported_err_to_variance(error, err_code):
    """Given an error expressed in a 95% confidence interval (from gpsd), return variance value.
    Must be resiliant to the error code not being reported, based on fix. Returns None in that case."""
    var = None
    error_val = error.get(err_code)
    if val is not None:
        #95% confidence interval, so within 2 standard deviations.
        #varance is stddev/sigma squared
        var = ((error_val / 2) ** 2)
    return var

def kalman_step(x, F, u, P, Z, H, R, I):
    """One Kalman Filter prediction/measurement iteration, where all parameters are numpy matrices/arrays."""
    # prediction
    x = (F * x) + u
    P = F * P * F.T
        
    # measurement update
    y = Z.T - (H * x)
    S = H * P * H.T + R
    K = P * H.T * S.I
    x = x + (K * y)
    P = (I - (K * H)) * P
    return x, P
        

if __name__ == '__main__':
    connect_local()
    
    #state vector while testing gps:
    #x = [x, y, altitude, dx, dy, horizontal speed (m/s), track]
    #local plane is tangent to earth at 0,0.  Positive y is North.
    #x has n elements.
    x = np.array([0., 0., 0., 0., 0., 0., 0.])
    
    #F - Next state update matrix, nxn
    F = np.matrix([[1.,0.,0.,1.,0.,0.,0.],
                   [0.,1.,0.,0.,1.,0.,0.],
                   [0.,0.,1.,0.,0.,0.,0.],
                   [0.,0.,0.,1.,0.,0.,0.],
                   [0.,0.,0.,0.,1.,0.,0.],
                   [0.,0.,0.,0.,0.,1.,0.],
                   [0.,0.,0.,0.,0.,0.,1.]])

    #measurement is like m = [x, y, altitude, dx, dy (computed from speed/track), speed, track] #and maybe drop speed and track here...
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
                   [0.,0.,0.,0.,0.,0.,1000.],])
    #R - covariance matrix of z measurement.  mxm
    #GPS datasheet says CEP is 2.5m.
    #CEP: Circular Error Probable, defined as the radius of a circle, centered on the mean, whose boundary is expected
    #to include 50% of the "landing points" (from ballistics usage).
    #From wikipedia: 0.674490 * stddev covers 50% of population within that confidence interval.
    #root-2 * sigma, that is.
    #So 0.674490 * stddev = 2.5m
    #stddev = 3.7065041735236993876855105338849, variance = 13.738173188348601860982040259622
    GPS_LOC_VARIANCE = 13.738173188348601860982040259622

    #GPS datasheet velocity says 0.1m/s accuracy.  What does that mean? One sigma?  Two sigma?
    #stddev = .1, or 2 * stddev = .1
    #so 0.01, or .0025
    GPS_V_VARIANCE = 0.1 ** 2
    #There is also measurement-by-measurement error information in the gps feed.  Which is better?
    #Note - the Kalman Filter does not update this itself as part of the loop.
    #maybe a good strategy is to use this base R, but update it with values from the measurements when we have them.
    #No data on how much the track may be off.  TODO figure out if I can calculate or measure this.
    GPS_TRACK_VARIANCE = 1.0
    R = np.matrix([[GPS_LOC_VARIANCE,0.,0.,0.,0.,0.,0.],
                   [0.,GPS_LOC_VARIANCE,0.,0.,0.,0.,0.],
                   [0.,0.,GPS_LOC_VARIANCE,0.,0.,0.,0.],
                   [0.,0.,0.,GPS_V_VARIANCE,0.,0.,0.],
                   [0.,0.,0.,0.,GPS_V_VARIANCE,0.,0.],
                   [0.,0.,0.,0.,0.,GPS_V_VARIANCE,0.],
                   [0.,0.,0.,0.,0.,0.,GPS_TRACK_VARIANCE],])

    #u is a placeholder for now...
    u = np.array([0.,0.,0.,0.,0.,0.,0.])
    #I is an identity matrix
    I = np.matrix([[1.,0.,0.,0.,0.,0.,0.],
                [0.,1.,0.,0.,0.,0.,0.],
                [0.,0.,1.,0.,0.,0.,0.],
                [0.,0.,0.,1.,0.,0.,0.],
                [0.,0.,0.,0.,1.,0.,0.],
                [0.,0.,0.,0.,0.,1.,0.],
                [0.,0.,0.,0.,0.,0.,1.],])
    
    set_origin()
    hack_time()

    last_gps_time = None

    #Main filter loop
    while True:
        start_loop_ts = time.time()
        p = gpsd.get_current()
        #ensure the reading is fresh
        while p.time == last_gps_time:
            time.sleep(0.05)
            p = gpsd.get_current()
        last_gps_time = p.time

        #measurement is like m = [x_pos, y_pos, altitude, dx, dy (computed from speed/track), speed, track] #and maybe drop speed and track here...
        x_pos,y_pos = coords_to_xy(p.lat, p.lon)
        #get dx,dy from p.hspeed and p.track
        dx,dy = bearing_dist_to_plane(radians(p.track), p.hspeed)
        #Decide which measurements are available based on fix mode.
        Hp = H.copy()
        if p.mode < 3:
            Hp[2][2] = 0.
        if p.mode < 2:
            Hp[0][0] = 0.
            Hp[1][1] = 0.
            Hp[3][3] = 0.
            Hp[4][4] = 0.
            Hp[5][5] = 0.
            Hp[6][6] = 0.
        #measurement array
        m = np.array([[x_pos, y_pos, p.altitude, dx, dy, p.hspeed, p.track]])

        #Update R covariance of measurements based on reported data if available
        Rp = R.copy()
        if p.mode >= 2:
            #x/logitude error
            x_var = reported_err_to_variance(p.error, 'epx')
            if x_var is not None:
                Rp[0][0] = x_var
            #y/latitude error
            y_var = reported_err_to_variance(p.error, 'epy')
            if y_var is not None:
                Rp[1][1] = y_var
            #vertical error
            v_var = reported_err_to_variance(p.error, 'epv')
            if v_var is not None:
                Rp[2][2] = v_var
            #speed error, m/s
            s_var = reported_err_to_variance(p.error, 'eps')
            if s_var is not None:
                #TODO technically, dx/dy are not EXACTLY this...
                Rp[3][3] = s_var
                Rp[4][4] = s_var
                Rp[5][5] = s_var

        x,P = kalman_step(x, F, u, P, Z, Hp, R, I)
        
        #Sleep for the rest of the second.
        end_loop_ts = time.time()
        sleep_time = 1.0 - end_loop_ts - start_loop_ts
        if sleep_time > 0:
            time.sleep(sleep_time)