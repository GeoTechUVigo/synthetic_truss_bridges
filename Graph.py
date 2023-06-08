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

import open3d as o3d
import numpy as np
from utils import angle_between_2Dvectors
from utils import angle_between_vectors
from utils import line_intersection


class Graph(object):


    def __init__(self, beam_ends, precision= 1/10**2) -> object:
        """
        Constructor of Graph class. A graph is created from a structural model.
        beams_ends must be centred in (0,0,0) and oriented in X.

        :param beam_ends: start point of the beam.
        """

        # check intersections between lines.
        intersections = np.ones((len(beam_ends), len(beam_ends),3)) * np.nan

        for i in range(len(beam_ends)):
            for j in range(len(beam_ends)):

                if i == j: continue

                # if both are diagonal continue
                if np.sum(beam_ends[i,0] - beam_ends[i,1] == 0) < 2 and np.sum(beam_ends[j,0] - beam_ends[j,1] == 0) < 2: continue

                # same x
                if np.absolute(beam_ends[i, 0, 0] - beam_ends[i, 1, 0]) < precision and np.absolute(beam_ends[j, 0, 0] - beam_ends[j, 1, 0]) < precision and np.absolute(beam_ends[i, 0, 0] - beam_ends[j, 0, 0]) < precision:
                    x,y = line_intersection(beam_ends[i][:,[0,1]], beam_ends[j][:,[0,1]], decimals=2)
                    if x is not None:
                        intersections[i,j] = beam_ends[i, 0, 0], x, y

                # same y
                if np.absolute(beam_ends[i, 0, 1] - beam_ends[i, 1, 1]) < precision and np.absolute(beam_ends[j, 0, 1] - beam_ends[j, 1, 1]) < precision and np.absolute(beam_ends[i, 0, 1] - beam_ends[j, 0, 1]) < precision:
                    x,y = line_intersection(beam_ends[i][:,[0,2]], beam_ends[j][:,[0,2]], decimals=2)
                    if x is not None:
                        intersections[i,j] = x, beam_ends[i, 0, 1], y

                # same z
                if np.absolute(beam_ends[i, 0, 2] - beam_ends[i, 1, 2]) < precision and np.absolute(beam_ends[j, 0, 2] - beam_ends[j, 1, 2]) < precision and np.absolute(beam_ends[i, 0, 2] - beam_ends[j, 0, 2]) < precision:
                    x,y = line_intersection(beam_ends[i][:,[0,2]], beam_ends[j][:,[0,2]], decimals=2)
                    if x is not None:
                        intersections[i,j] = x, y, beam_ends[i, 0, 2]

        
        # set nodes
        a = intersections.reshape(-1,3)
        b = a[~np.isnan(a[:,0])]
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(b)
        o3d.visualization.draw([pcd])


