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


def line_intersection(line1, line2):
    '''
    Method for computing the point interseciton between 2 lines in 2D.

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
       raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y