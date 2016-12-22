import pynmea2
from pynmea2.nmea import ParseError

#A rather nice, uncouth writeup about why GPS sucks: http://esr.ibiblio.org/?p=801

#looks like my GPS reciever's burst starts with $GPRMC and ends with $GPGLL.
#every GPS can be different. Mine is a GP-20U7 from Sparkfun.

#RMC: time/date(2-digit year)?  lat, lng, speed, course made good (sort of but not exactly heading - can i get this from IMU instead?)
#VTG: Track made good - should be the same as CMG above. Speed again in knots and km/h.
#GGA: Fix quality: 0 no fix, 1 "GPS - 15m, 2 "Differential GPS" up to 10cm.  Altitude in meters above mean sea level.  Height of geoid above WGS84 ellipsoid.  These may or may NOT be present - test when you have a fix.
#GSA: GPS DOP and active satellites - nothing, except for maybe mode (2d, 3d - modes are confusing). Unless I figure out how to use DOPs.
#GSV: GPS Satellites in view - nothing
#GLL: A or V for valid or invalid data. What does this mean?

tick = 0
FIRST_MSG = "RMC"
LAST_MSG = "GLL"

def handle_message(m):
    """Update state with a parsed nmea gps message"""
    if m.sentence_type == FIRST_MSG:
        ticks += 1


def read(filename):
    print("starting read")
    f = open(filename)
    for line in f:
        try:
            parsed = pynmea2.parse(line)
            print(parsed)
        except:
            print("bad gps read...")


if __name__ == '__main__':
    read('gps_test_output.txt')