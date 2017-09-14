import unittest
from coordinate_utils import *

class Test_coord_utils(unittest.TestCase):
    def test_coord_utils(self):
        latLA = 34.00000048
        lonLA = -117.3335693
        hLA = 251.702
        #x0 = y0 = z0 = 0
        x0, y0, z0 = GeodeticToEcef(latLA, lonLA, hLA)

        self.assertAlmostEqual(-2430601.8, x0)
        self.assertAlmostEqual(-4702442.7, y0)
        self.assertAlmostEqual(3546587.4, z0)

        #Checks to read out the matrix entries, to compare with the paper
        #First column
        x = x0 + 1
        y = y0
        z = z0
        xEast, yNorth, zUp = EcefToEnu(x, y, z, latLA, lonLA, hLA)
        self.assertAlmostEqual(0.88834836, xEast)
        self.assertAlmostEqual(0.25676467, yNorth)
        self.assertAlmostEqual(-0.38066927, zUp)

        x = x0
        y = y0 + 1
        z = z0
        xEast, yNorth, zUp = EcefToEnu(x, y, z, latLA, lonLA, hLA)
        self.assertAlmostEqual(-0.45917011, xEast)
        self.assertAlmostEqual(0.49675810, yNorth)
        self.assertAlmostEqual(-0.73647416, zUp)

        x = x0
        y = y0
        z = z0 + 1
        xEast, yNorth, zUp = EcefToEnu(x, y, z, latLA, lonLA, hLA)
        self.assertAlmostEqual(0.00000000, xEast)
        self.assertAlmostEqual(0.82903757, yNorth)
        self.assertAlmostEqual(0.55919291, zUp)

if __name__ == '__main__':
    unittest.main()