
import numpy as np
import math
import matplotlib.pyplot as plt

"""
Python file containing the functions to create the radar bins, used in the plotter app. 
"""

AMOUNT_POINTS = 100


def distance(xPtA, yPtA, xPtB, yPtB):
    """
        Returns the euklid distance of two points
    """
    return np.sqrt((xPtA-xPtB)**2 + (yPtA-yPtB)**2)


def getPolarBounds(xP, yP):
    """
        Returns the uper and lower polar coordinates for the radar bin, used in the animation plotter app
    """
    rP = distance(xP, yP, 0, 0)
    phiP = math.atan2(yP, xP)
    phiP = phiP*180/math.pi
    RANGE_PRECISION = 0.043
    ANGLE_PRECISION = 3.75
    rLower = rP-(rP % RANGE_PRECISION)
    rUpper = rLower+RANGE_PRECISION
    phiLower = phiP-(phiP % ANGLE_PRECISION)
    phiUpper = phiLower+ANGLE_PRECISION
    #print(f"rP:{rP} rLower:{rLower}, rUpper:{rUpper}")
    #print(f"phiP:{phiP} phiLower:{phiLower}, phiUpper:{phiUpper}")
    return rLower, rUpper, phiLower, phiUpper


def getPolarBoundsOffset(xP, yP, xOffset):
    """
        Get the polar coordinates of the radar bins, corrected with the offset
    """
    return getPolarBounds(xP-xOffset, yP)


def polar2z(r, theta):
    """
        Returns the cartesian coordninates of polar coordinates
    """
    return r * np.exp(1j * theta)


def getRadarBinOffset(pointX, pointY, radaroffsetX):
    """
        Returns the offset of a radar bin (Stützvektor)
    """
    rLower, rUpper, phiLower, phiUpper = getPolarBoundsOffset(
        pointX, pointY, radaroffsetX)
    phi = np.linspace(phiLower, phiUpper, AMOUNT_POINTS)
    r = np.linspace(rLower, rUpper, AMOUNT_POINTS)
    x = []
    y = []
    for pVal in phi:
        for rVal in r:
            x.append((rVal*np.cos(pVal*np.pi/180))+radaroffsetX)
     #       print(f"pVAL{pVal}, rVal{rVal}")
            y.append(rVal*np.sin(pVal*np.pi/180))

    return x, y


def getRadarBin(pointX, pointY):
    """
        Return the cartesian coordinates, which defines the radar bin
    """
    rLower, rUpper, phiLower, phiUpper = getPolarBounds(pointX, pointY)
    phi = np.linspace(phiLower, phiUpper, AMOUNT_POINTS)
    r = np.linspace(rLower, rUpper, AMOUNT_POINTS)
    x = []
    y = []
    for pVal in phi:
        for rVal in r:
            x.append(rVal*np.cos(pVal*np.pi/180))
     #       print(f"pVAL{pVal}, rVal{rVal}")
            y.append(rVal*np.sin(pVal*np.pi/180))

    return x, y


def getIntesection(xs1, ys1, xs2, ys2):
    """
        Calculate and return the intersection area of two radar bins
    """
    index = 0
    xys1 = []
    xys2 = []
    for x in xs1:
        xys1.append((round(x, 4), round(ys1[index], 4)))
        index += 1
    index = 0
    for x in xs2:
        xys2.append((round(x, 4), round(ys2[index], 4)))
        index += 1
    xys1 = set(xys1)
    xys2 = set(xys2)
    xyIntersect = xys1.intersection(xys2)
    xInt = []
    yInt = []
    for el in xyIntersect:
        xInt.append(el[0])
        yInt.append(el[1])
    return xInt, yInt
