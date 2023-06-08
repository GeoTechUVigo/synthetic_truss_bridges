"""
Copyright (C) 2023 GeoTECH Group <geotech@uvigo.gal>
Copyright (C) 2023 Daniel Lamas Novoa <daniel.lamas.novoa@uvigo.gal> <https://orcid.org/0000-0001-7275-183X>
Copyright (C) 2023 Andrés Justo Domínguez <andres.justo.dominguez@uvigo.gal> <https://orcid.org/0000-0003-2072-4076>
This file is part of the program synthetic_truss_bridges.
The program is free software: you can redistribute it and/or modify it 
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 3 of the License, or any later version.
The program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or 
FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
more details.
You should have received a copy of the GNU General Public License along 
with the program in COPYING. If not, see <https://www.gnu.org/licenses/>. 
"""

import numpy as np


def angle_between_vectors(a: np.ndarray, b: np.ndarray) -> float:
    """
    Calculates the angle between the vector a and b.

    :param a: (N,) numpy.ndarray numeric.
    :param b: (N,) numpy.ndarray numeric.
    """

    # Reshaping inputs
    a = np.asarray(a).reshape(-1)
    b = np.asarray(b).reshape(-1)

    inner = np.inner(a, b)
    norms = np.linalg.norm(a) * np.linalg.norm(b)

    if norms == 0:
        rad = np.nan

    else:
        cos = inner / norms
        rad = np.arccos(np.clip(cos, -1.0, 1.0))

    return rad


def angle_between_2Dvectors(a,b) -> float:
    """
    Calculates the angle between from vector a to the vector b, being both 2D vectors. 
    The sense of the angle is considered. The output is in radians 0-2pi.

    :param a: (2,) numpy.ndarray numeric.
    :param b: (2,) numpy.ndarray numeric.
    """

    # Reshaping inputs
    a = np.asarray(a).reshape(-1)
    b = np.asarray(b).reshape(-1)

    # a angle with X axis
    a_rad = angle_between_vectors(a,[1,0])

    # Check direction
    if a[1] < 0:
        a_rad = 2*np.pi - a_rad

    # b angle with X axis
    b_rad = angle_between_vectors(b,[1,0])

    # Check direction
    if b[1] < 0:
        b_rad = 2*np.pi - b_rad


    angle_ab = b_rad - a_rad
    if angle_ab < 0:
        angle_ab = 2*np.pi + angle_ab

    # print(np.rad2deg(angle_ab))

    return angle_ab


def line_intersection(line1, line2, decimals=2):
    '''
    Method for computing the point interseciton between 2 segments in 2D.

    :param line1: tuple  with the endpoints of the line.
    :param line2: tuple  with the endpoints of the line.
    :return x,y
    '''
    
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return None, None

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    # If the intersection if out of segment limits
    max_x = np.ceil(np.max((line1[:,0],line2[:,0]))*10**decimals)/10**decimals
    min_x = np.floor(np.min((line1[:,0],line2[:,0]))*10**decimals)/10**decimals

    max_y = np.ceil(np.max((line1[:,1],line2[:,1]))*10**decimals)/10**decimals
    min_y = np.floor(np.min((line1[:,1],line2[:,1]))*10**decimals)/10**decimals

    x_ = np.round(x, decimals=decimals)
    y_ = np.round(y, decimals=decimals)
    if x_ > max_x or x_ < min_x or y_ > max_y or y_ < min_y:

        return None, None
    
    return x, y

def line_intersection_3d(A, B):
    '''
    Method for computing the point interseciton between 2 segments in 2D.

    :param line1: tuple  with the endpoints of the line.
    :param line2: tuple  with the endpoints of the line.
    :return x,y
    '''

    A = np.array([[4, 0, 0], [4, -3, 0]])
    B = np.array([[6, 2, 0], [10, 2, 0]])
    if np.linalg.det(np.array([A[1]-A[0], B[0]-B[1]]).T) == 0: return None, None
    t, s = np.linalg.solve(np.array([A[1]-A[0], B[0]-B[1]]).T, B[0]-A[0])

    return (1-s)*B[0] + s*B[1]

def in_line(line_ends, points):
    '''
    Method for checking which points are part of the line.

    :param line1: endpoints of the lines.
    :param points: points to be checked.
    :return output
    '''

    output = np.ones(len(points),dtype=np.bool_)
    
    slope = line_ends[1] - line_ends[0]

    mask = slope != 0 


    results = (points[:,mask] - line_ends[0][mask]) / slope[mask]
    
    # If there are in the line, the result of the equation of each dimension must be the same
    for i in range(results.shape[1]):
        output = np.logical_and((results[:,0] - results[:,i])< 0.01, output)

    # If slope == 0 in any dimensions, x=0 == x
    if not np.all(mask):
        for i in range(results.shape[1]):
            if not mask[i]:
                output = np.logical_and((line_ends[0,i] == points[:,i]), output)

    return output