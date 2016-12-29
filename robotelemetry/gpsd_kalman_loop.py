import gpsd
import time
import numpy as np
from math import radians, sin, cos, sqrt, asin, atan2
from flask_socketio import SocketIO

def connect_local():
    gpsd.connect()

def coords_to_xy(lat, lon):
    """Converts latitude/longitude coordinates to local x/y plane, using Haversine"""
    R = 6372.8 * 1000 #in meters
    if ORIGIN_LOC:
        #from origin (1) to point (2)
        dLat = radians(lat - ORIGIN_LOC[0])
        dLon = radians(lon - ORIGIN_LOC[1])
        rlat1 = radians(ORIGIN_LOC[0])
        rlat2 = radians(lat)
        a = sin(dLat/2)**2 + cos(rlat1)*cos(rlat2)*sin(dLon/2)**2
        c = 2*asin(sqrt(a))
        dist = R*c
        #Now, if I knew what direction...
        #initial bearing - over long distances, a straight line may change.
        y1 = sin(dLon) * cos(rlat2);
        x1 = cos(rlat1)*sin(rlat2) - sin(rlat1)*cos(rlat2)*cos(dLon);
        brng = atan2(y1, x1); #from origin, in radians
        #to x/y
        x = Cos(brng) * dist
        y = Sin(brng) * dist
        return (x,y)

def kalman_step(x):
    pass

if __name__ == '__main__':
    HAS_FIX = False
    ORIGIN_LOC = None

    connect_local()
    p = gpsd.get_current()
    while p.mode < 2:
        p = gpsd.get_current()
        if p.mode == 3:
            HAS_FIX = True
            ORIGIN_LOC = (p.lat, p.lon)
        time.sleep(1)

    #while True:
    #    p = gpsd.get_current()
    #    pass

    #state vector while testing gps:
    #x = [lat, lng, altitude, horizontal speed, track]
    x = np.array([p.lat, p.lon, p.alt, p.hspeed, p.track])
    #or alternatively...  set first fix lat/lon as (0,0) in x/y plane, treat everything as 2d from there.
    #local plane is tangent to earth at 0,0.  Positive y is North.
    #x = [x, y, altitude, horizontal speed (m/s), track]

    #x = np.array([])

    kalman_step(x)