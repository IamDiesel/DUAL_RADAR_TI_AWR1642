import math
import numpy as np

"""
    Python module, which contains two classes to describe the detected points and clusters
"""


class DetectedPoints:
    """
        List for detected points
    """

    def __init__(self, xs=None, ys=None, vs=None, peakVals=None, origin=None):
        """
            Init function
        """
        if(xs is None):
            self.xs = []
        else:
            self.xs = xs

        if(ys is None):
            self.ys = []
        else:
            self.ys = ys

        if(vs is None):
            self.vs = []
        else:
            self.vs = vs

        if(peakVals is None):
            self.peakVals = []
        else:
            self.peakVals = peakVals

        if(origin is None):
            self.origins = []
        else:
            self.origins = origin

    def extendAll(self, xs, ys, vs, peakVals, origins):
        """
            Extend the list by the parameters
        """
        if(not (any(x is None for x in xs) or any(y is None for y in ys) or any(v is None for v in vs) or any(p is None for p in peakVals))):
            self.xs.extend(xs)
            self.ys.extend(ys)
            self.vs.extend(vs)
            self.peakVals.extend(peakVals)
            self.origins.extend(origins)

    def appendX(self, x):
        """
            Append a x coordinate to the list
        """
        if x is not None:
            self.xs.append(x)

    def appendY(self, y):
        """
            Append a y coordinate to the list
        """
        if y is not None:
            self.ys.append(y)

    def appendV(self, v):
        """
            Append a speed value to the list
        """
        if v is not None:
            self.vs.append(v)

    def appendPeakVal(self, peakVal):
        """
            Append a peak value to the list
        """
        if peakVal is not None:
            self.peakVals.append(peakVal)

    def appendOrigins(self, ori):
        """
            Append the origin (radar 1 or 2) to the list
        """
        if ori is not None:
            self.origins.append(ori)

    def isEmpty(self):
        """
            Check if the list is empty
        """
        """print(f'xs:{self.xs} ys:{self.ys} vs:{self.vs} peaks:{self.peakVals}')
        if(any(x is None for x in self.xs)):
            print("x is none")
        if(any(y is None for y in self.ys)):
            print("y is none")
        if(any(v is None for v in self.vs)):
            print("v is none")
        if(any(p is None for p in self.peakVals)):
            print("peak is none")
        print(f'len:{(len(self.xs) + len(self.ys) + len(self.vs) + len(self.peakVals))}')"""

        if((len(self.xs) + len(self.ys) + len(self.vs) + len(self.peakVals)) < 1 or any(x is None for x in self.xs) or any(y is None for y in self.ys) or any(v is None for v in self.vs) or any(p is None for p in self.peakVals)):
            return True
        else:
            return False

    def getXYArray(self):
        """
            Returns a array containing all x and y coordinates of the list
        """
        index = 0
        res = []
        for elem in self.xs:
            if(elem is not None):
                tmpElem = []
                tmpElem.append(elem)
                tmpElem.append(self.ys[index])
                res.append(tmpElem)
            index += 1
        return res

    def setAllOrigin(self, val):
        """
            Set the origin of all elements to the val parameter
        """
        index = 0
        self.origins = []
        for i in self.xs:
            self.origins.append(val)
            index += 1

    def getXYMaxPeakVal(self, ptsIndexList):
        """
            Returns the x/y coordinates of the points with the highest peak value in the list
        """
        x = None
        y = None
        v = None
        maxPeakVal = 0
        for i in ptsIndexList:
            if(self.peakVals[i] > maxPeakVal):
                maxPeakVal = self.peakVals[i]
                x = self.xs[i]
                y = self.ys[i]
                v = self.vs[i]
        return (x, y, v, maxPeakVal)

    def extendFromOtherDetectedObject(self, other):
        """
            Extend the list with values from the "other" parameter
        """
        if(not (any(x is None for x in other.xs) or any(y is None for y in other.ys) or any(v is None for v in other.vs) or any(p is None for p in other.peakVals))):
            self.xs.extend(other.xs)
            self.ys.extend(other.ys)
            self.vs.extend(other.vs)
            self.peakVals.extend(other.peakVals)
            self.origins.extend(other.origins)

    def getR1R2DataXY(self):
        """
            Return 6 list, 3 for each radar containing the coordinates and the peak values
        """
        xs1 = []
        xs2 = []
        ys1 = []
        ys2 = []
        pks1 = []
        pks2 = []
        index = 0
        for x in self.xs:

            if(self.origins[index] == 0):
                xs1.append(x)
                ys1.append(self.ys[index])
                pks1.append(self.peakVals[index])
            else:
                xs2.append(x)
                ys2.append(self.ys[index])
                pks2.append(self.peakVals[index])
            index += 1
        return (xs1, ys1, pks1, xs2, ys2, pks2)

    def getPeakValueWeightedCenterOfGravity(self):
        """
            Returns the center of gravity of the detected points, which are weighted by the peak value
        """
        index = 0
        wSumX = 0
        wSumY = 0
        sumPeakVal = 0
        for peakVal in self.peakVals:
            sumPeakVal += peakVal

        # weighted arithmetic mean = 1/n * sum(x_i * w_i)
        for x in self.xs:
            wSumX += self.peakVals[index]/sumPeakVal * x
            wSumY += self.peakVals[index]/sumPeakVal * self.ys[index]
            index += 1

        return (wSumX, wSumY)

    def getArithmeticMean(self):
        """
            Get the arithmetic mean of the coordinates
        """
        index = 0
        sumX = 0
        sumY = 0

        # weighted arithmetic mean = 1/n * sum(x_i * w_i)
        for x in self.xs:
            sumX += x
            sumY += self.ys[index]
            index += 1
        return (sumX/index, sumY/index)


class DetectedClusters:
    """
        List for the detected clusters
    """

    def __init__(self, xsCenter=None, ysCenter=None, xsSize=None, ysSize=None):
        """
            Init function
        """
        if(xsCenter is None):
            self.xsCenter = []
        else:
            self.xsCenter = xsCenter

        if(ysCenter is None):
            self.ysCenter = []
        else:
            self.ysCenter = ysCenter

        if(xsSize is None):
            self.xsSize = []
        else:
            self.xsSize = xsSize

        if(ysSize is None):
            self.ysSize = []
        else:
            self.ysSize = ysSize

    def appendXCenter(self, xCenter):
        """
            Append the x coordinate of a cluster
        """
        self.xsCenter.append(xCenter)

    def appendYCenter(self, yCenter):
        """
            Append the x coordinate of a cluster
        """
        self.ysCenter.append(yCenter)

    def appendXSize(self, xSize):
        """
            Append the size/length in x direction of a cluster
        """
        self.xsSize.append(xSize)

    def appendYSize(self, ySize):
        """
            Append the size/length in y direction of a cluster
        """
        self.ysSize.append(ySize)

    def isEmpty(self):
        """
            Check, if the list is empty
        """
        if((len(self.xsCenter) + len(self.ysCenter) + len(self.xsSize) + len(self.ysSize)) < 1):
            return True
        else:
            return False
