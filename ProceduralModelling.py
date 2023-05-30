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
from TrussBridge import TrussBridge


class ProceduralModelling(TrussBridge):


    def __init__(self, n_drawers:int, height:float, length:float, width:float, h_deck:float, centre=[0,0,0], orientation=[0,0,0],
                 chord:list = None, diagonal_vert:list = None, parallel_vert:list = None, diagonal_bottom:list = None, 
                 parallel_bottom:list = None, diagonal_top:list = None, parallel_top:list = None, diagonal_inner:list = None,
                 density=None, cameras=None) -> TrussBridge:
        """
        Class child of TrussBridge. 
        This constructor calculates the node_coordinates and the parametres of TrussBridge object based on the Brown Truss structure.
        The orientation of each beam specifies the last rotation. These rotations are done in the class Beam.
        The rotations place the object from the object extruded in Z to the direction node0-node1.
        The order of rotation is ZYZ.
        The nodes of each beam (node0-node1) are sorted as follows: node1_x > node0_x. If =, node1_0 > node0_y. 
        Except for the vertical face at +Y, which is mirrored (node1-node0).

        :param n_drawers: number of drawers.
        :param height: height of the drawer.
        :param length: lenght of the drawer.
        :param width: width of the bridge.
        :param centre: (3,) centre of the Truss.
        :param orientation: (3,) orientation of the Truss expresed as ZYX rotation angles.
        :param h_deck: [height, width] lower height position of the deck and its width.
        :param chord: [string, doulbe, uint] profile of chord beams,its orientation and the class number.
        :param diagonal_vert: [string, doulbe, uint] profile of vertical diagonal beams,its orientation and the class number.
        :param parallel_vert: [string, doulbe, uint] profile of vertical parallel beams,its orientation and the class number.
        :param diagonal_bottom: [string, doulbe, uint] profile of bottom diagonal beams,its orientation and the class number.
        :param parallel_bottom: [string, doulbe, uint] profile of bottom parallel beams,its orientation and the class number.
        :param diagonal_top: [string, doulbe, uint] profile of top diagonal beams,its orientation and the class number.
        :param parallel_top: [string, doulbe, uint] profile of top parallel beams,its orientation and the class number.
        :param diagonal_inner: [string, doulbe, uint] profile of inner diagonal beams,its orientation and the class number.
        :param density: density of the point cloud (points/m²).
        :param cameras: list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px']. Consider the point centre in (0,0,0)
        """