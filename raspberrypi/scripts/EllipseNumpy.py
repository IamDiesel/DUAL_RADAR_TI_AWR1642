"""
Functions to create the uncertainty ellipses used in the plotter app
"""
import numpy as np
from shapely.geometry.polygon import LinearRing
import logging


def ellipse_polyline(ellipses, n=1000):
    """
        Returns sample points of the discretized ellipse
    """
    t = np.linspace(0, 2*np.pi, n, endpoint=False)
    st = np.sin(t)
    ct = np.cos(t)
    result = []
    for x0, y0, a, b, angle in ellipses:
        angle = np.deg2rad(angle)
        sa = np.sin(angle)
        ca = np.cos(angle)
        p = np.empty((n, 2))
        p[:, 0] = x0 + a * ca * ct - b * sa * st
        p[:, 1] = y0 + a * sa * ct + b * ca * st
        result.append(p)
    return result


def intersections(a, b):
    """
        Returns the intersection points of two ellipses
    """
    intersection_points = []
    ea = LinearRing(a)
    eb = LinearRing(b)
    mp = ea.intersection(eb)
    try:
        x = [p.x for p in mp]
        y = [p.y for p in mp]
        for q in range(0, len(x)):
            intersection_points.append((x[q], y[q]))
    except Exception as e:
        logging.debug(e)
        intersection_points.append((0, 0))
    return intersection_points
