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
import copy

class TrussModelling(TrussBridge):


    def __init__(self, length:float, height:float, width:float, h_deck:float, centre=[0,0,0], orientation=[0,0,0],
                 chord:list = None, diagonal_vert:list = None, parallel_vert:list = None, diagonal_bottom:list = None, parallel_bottom:list = None, 
                 diagonal_top:list = None, parallel_top:list = None, diagonal_inner:list = None, parallel_inner:list = None,
                 density=None, cameras=None) -> TrussBridge:
        """
        Class child of TrussBridge. 
        This constructor calculates the node_coordinates and the parametres of TrussBridge object based on the input parameters,
        based on procedural modelling.
        The orientation of each beam specifies the last rotation. These rotations are done in the class Beam.
        The rotations place the object from the object extruded in Z to the direction node0-node1.
        The order of rotation is ZYZ.
        The nodes of each beam (node0-node1) are sorted as follows: node1_x > node0_x. If =, node1_0 > node0_y. 
        Except for the vertical face at +Y, which is mirrored (node1-node0).

        :param n_panels: number of panels.
        :param height: height of the panel.
        :param length: lenght of the panel.
        :param width: width of the bridge.
        :param centre: (3,) centre of the Truss.
        :param orientation: (3,) orientation of the Truss expresed as ZYX rotation angles.
        :param h_deck: [height, width] lower height position of the deck and its width.
        :param chord: [string, doulbe, uint] number of chords/face, profile of chord beams,its orientation and the class number.
        :param diagonal_vert: [string, doulbe, uint] profile of vertical diagonal beams,its orientation and the class number.
        :param parallel_vert: [string, doulbe, uint] profile of vertical parallel beams,its orientation and the class number.
        :param diagonal_bottom: [string, doulbe, uint] profile of bottom diagonal beams,its orientation and the class number.
        :param parallel_bottom: [string, doulbe, uint] profile of bottom parallel beams,its orientation and the class number.
        :param diagonal_top: [string, doulbe, uint] profile of top diagonal beams, its orientation and the class number.
        :param parallel_top: [string, doulbe, uint] profile of top parallel beams, its orientation and the class number.
        :param diagonal_inner: [string, double, uint] profile of inner diagonal beams, its orientation and the class number.
        :param parallel_inner: [uint, string, double, uint] number of parallel beams by inner face, its profile, its orientation and the class number.
        :param density: density of the point cloud (points/m²).
        :param cameras: list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px']. Consider the point centre in (0,0,0)
        """

        # Create faces in XY plane and then place it.

        #############################################################################################
        # Chords
        #############################################################################################

        beam_nodes = np.array([[0,1],[2,3],[5,4],[7,6]]) # Mirror orientation.
        beam_sect = np.asarray(chord[0])*np.ones(len(beam_nodes),dtype='object')
        beam_orient = np.asarray(chord[1]*np.ones(len(beam_nodes)))
        beam_sem = np.asarray(chord[2])*np.ones(len(beam_nodes), dtype='uint')

        node_coordinates = np.array([[0, 0, 0],[length, 0, 0],
                                     [0, 0, height],[length, 0, height],
                                     [0, width, 0],[length, width, 0],
                                     [0, width, height],[length, width, height]])



            
            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, parallel_inner[1:])

        ###########################################################################################################################
        # Remove repited nodes.
        node_coordinates, order = np.unique(node_coordinates, return_inverse=True, axis=0)
        beam_nodes = order[beam_nodes]

        ###########################################################################################################################
        # deck
        deck_points = np.asarray([[0, width/2, h_deck[0]+h_deck[1]/2],[length*n_panels, width/2, h_deck[0]+h_deck[1]/2]])

        ###########################################################################################################################
        # Move Truss to the 0,0,0
        node_coordinates = node_coordinates - [n_panels*length/2, width/2, height/2]
        deck_points = deck_points - [n_panels*length/2, width/2, height/2]

        deck = [deck_points[0], deck_points[1], [width, h_deck[1]]]

        # TrussBridge constructor with the elements of BaileyTrusss
        super().__init__(node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem, deck, centre, orientation, density, cameras)


    @staticmethod
    def horizontal_beams(n_beams, plane, length, height, ends, ratio:float = 1):
        
        n_beams = n_beams if ends else n_beams+2

        beam_p0 = np.concatenate((np.zeros([n_beams,1]),
                                  np.zeros([n_beams,1]), 
                                  (np.arange(n_beams) * height).reshape(-1,1)
                                  ), axis=1)
        
        beam_p1 = np.concatenate((np.ones([n_beams,1]) * length,
                                  np.zeros([n_beams,1]), 
                                  (np.arange(n_beams) * height).reshape(-1,1)
                                  ), axis=1)
        
        if not ends:
           beam_p0, beam_p1 = TrussModelling.remove_ends(np.asarray(beam_p0, beam_p1))

        beam_p0, beam_p1 = TrussModelling.change_plane(plane)

        return beam_p0, beam_p1
    

    @staticmethod
    def vertical_beams(n_beams, plane, length, height, ends, ratio:float = 1):



    @staticmethod
    def diag_downer_beams():
        

    @staticmethod
    def diag_upper_beams():


    @staticmethod
    def change_plane(beam_p0, beam_p1, plane):
        """
        The original plane must be XZ
        """

        if plane == 'XY':
            beam_p0[:,[1,2]] = beam_p0[:, [2,1]]
            beam_p1[:,[1,2]] = beam_p1[:, [2,1]] 

        elif plane == 'YZ':
            beam_p0[:,[0,1]] = beam_p0[:, [1,0]]
            beam_p1[:,[0,1]] = beam_p1[:, [1,0]] 

        elif plane != 'XZ':
            TypeError('Incorrect plane')

        return beam_p0, beam_p1

    @staticmethod
    def mirrow():


    @staticmethod
    def remove_ends(arrays):
        '''
        Delete first and last element of the 2nd dimension of the array
        '''

        return arrays[:, 1:-1]