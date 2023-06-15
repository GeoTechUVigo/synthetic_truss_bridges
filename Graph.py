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
                    x,y = line_intersection(beam_ends[i][:,[1,2]], beam_ends[j][:,[1,2]], decimals=2)
                    if x is not None:
                        intersections[i,j] = beam_ends[i, 0, 0], x, y

                # same y
                if np.absolute(beam_ends[i, 0, 1] - beam_ends[i, 1, 1]) < precision and np.absolute(beam_ends[j, 0, 1] - beam_ends[j, 1, 1]) < precision and np.absolute(beam_ends[i, 0, 1] - beam_ends[j, 0, 1]) < precision:
                    x,y = line_intersection(beam_ends[i][:,[0,2]], beam_ends[j][:,[0,2]], decimals=2)
                    if x is not None:
                        intersections[i,j] = x, beam_ends[i, 0, 1], y

                # same z
                if np.absolute(beam_ends[i, 0, 2] - beam_ends[i, 1, 2]) < precision and np.absolute(beam_ends[j, 0, 2] - beam_ends[j, 1, 2]) < precision and np.absolute(beam_ends[i, 0, 2] - beam_ends[j, 0, 2]) < precision:
                    x,y = line_intersection(beam_ends[i][:,[0,1]], beam_ends[j][:,[0,1]], decimals=2)
                    if x is not None:
                        intersections[i,j] = x, y, beam_ends[i, 0, 2]

        
        # set nodes
        intersections_ = np.ones((len(beam_ends), len(beam_ends))) * np.nan
        nodes = np.zeros((0,3))
        for i in range(len(intersections)):
            for j in range(len(intersections[i])):
                node = intersections[i,j]

                if np.all(np.isnan(node)): continue

                new=True
                for k in range(len(nodes)):
                    if np.all(np.absolute(node - nodes[k]) < precision):
                        new=False
                        break
                
                if new == True:
                    nodes = np.concatenate((nodes, node.reshape(1,-1)), axis=0)
                    intersections_[i,j] = len(nodes)-1
                else:
                    intersections_[i,j] = k

        # Edges
        edges = np.zeros((0,3),dtype=np.int_)
        for i in range(len(intersections)):

            # nodes of this bar
            node_idx = np.unique(intersections_[i])
            node_idx = node_idx[~np.isnan(node_idx)].astype(np.int_)

            # sort nodes by distance to the start node of the bar
            order = np.argsort(np.linalg.norm(beam_ends[i][0] - nodes[node_idx], axis=1))
            node_idx = node_idx[order]

            start_node = node_idx[0]
            for j in range(len(node_idx)-1):
                end_node = node_idx[j+1]
                this_edge = np.array([[start_node, end_node, i]])
                edges = np.concatenate((edges,this_edge), axis=0)
                start_node = end_node

        self.nodes = nodes
        self.edges = edges


    def get_pcd(self):

        nodes = o3d.geometry.PointCloud()
        nodes.points = o3d.utility.Vector3dVector(self.nodes)

        edges = o3d.geometry.LineSet()
        edges.points = o3d.utility.Vector3dVector(self.nodes)
        edges.lines = o3d.utility.Vector2iVector(self.edges[:,0:2])

        return nodes, edges


    def show(self):
        
        nodes, edges = self.get_pcd()
    
        o3d.visualization.draw([nodes, edges])


