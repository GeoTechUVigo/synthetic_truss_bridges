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


class BaileyTruss(TrussBridge):


    def __init__(self, n_drawers:int, height:float, length:float, width:float, h_deck:float, centre=[0,0,0], orientation=[0,0,0],
                 chord:list = None, diagonal_vert:list = None, parallel_vert:list = None, diagonal_bottom:list = None, parallel_bottom:list = None, 
                 diagonal_top:list = None, parallel_top:list = None, diagonal_inner:list = None, parallel_inner:list = None,
                 density=None, cameras=None) -> TrussBridge:
        """
        Class child of TrussBridge. 
        This constructor calculates the node_coordinates and the parametres of TrussBridge object based on the Bailey Truss structure.
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
        :param diagonal_top: [string, doulbe, uint] profile of top diagonal beams, its orientation and the class number.
        :param parallel_top: [string, doulbe, uint] profile of top parallel beams, its orientation and the class number.
        :param diagonal_inner: [string, double, uint] profile of inner diagonal beams, its orientation and the class number.
        :param parallel_inner: [uint, string, double, uint] number of parallel beams by inner face, its profile, its orientation and the class number.
        :param density: density of the point cloud (points/m²).
        :param cameras: list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px']. Consider the point centre in (0,0,0)
        """

        #############################################################################################
        # Chords
        #############################################################################################

        beam_nodes = np.array([[0,1],[2,3],[5,4],[7,6]]) # Mirror orientation.
        beam_sect = np.asarray(chord[0])*np.ones(len(beam_nodes),dtype='object')
        beam_orient = np.asarray(chord[1]*np.ones(len(beam_nodes)))
        beam_sem = np.asarray(chord[2])*np.ones(len(beam_nodes), dtype='uint')

        node_coordinates = np.array([[0, 0, 0],[length*n_drawers, 0, 0],
                                     [0, 0, height],[length*n_drawers, 0, height],
                                     [0, width, 0],[length*n_drawers, width, 0],
                                     [0, width, height],[length*n_drawers, width, height]])

        #############################################################################################
        # Vertical faces
        #############################################################################################
        ## Diagonals
        if isinstance(diagonal_vert, type(None)):
            diagonal_vert = None
        else:
            
            # First and last point of each diagonal
            # Left to right diagonals in face XZ
            n_beams = n_drawers + 1

            beam_p0 = np.concatenate(((np.arange(n_beams) * length - length/2).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.zeros([n_beams,1])), axis=1)

            beam_p1 = np.concatenate(((np.arange(n_beams) * length + length/2).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.ones([n_beams,1]) * height), axis=1)

            # Correcting first and last diagonal
            beam_p0[0,:] = [0.0, 0.0, height/2]
            beam_p1[-1,:] = [length * n_drawers, 0.0, height/2]

            # Adding diagonals right to left
            beam_p0 = np.concatenate((beam_p0,
                                      np.concatenate(((np.arange(n_beams) * length - length/2).reshape(-1,1),
                                                       np.zeros([n_beams,1]), 
                                                       np.ones([n_beams,1]) * height), axis=1)), axis=0)

            beam_p1 = np.concatenate((beam_p1,
                                      np.concatenate(((np.arange(n_beams) * length + length/2).reshape(-1,1),
                                                       np.zeros([n_beams,1]), 
                                                       np.zeros([n_beams,1])), axis=1)), axis=0)

            # Correcting first and last diagonal
            beam_p0[n_beams,:] = [0.0, 0.0, height/2]
            beam_p1[-1,:] = [length * n_drawers, 0.0, height/2]
            
            # Updating number of diagonals 
            n_beams = len(beam_p0)

            # Adding the diagonals in the other face. Mirror orientation
            beam_p0, beam_p1 = np.concatenate((beam_p0, beam_p1),axis=0), np.concatenate((beam_p1, beam_p0),axis=0)
            # Changing Y dimension
            beam_p0[n_beams:, 1] = width
            beam_p1[n_beams:, 1] = width

            # Updating number of diagonals 
            n_beams = len(beam_p0)

            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, diagonal_vert)

        ## Parallels
        if isinstance(parallel_vert, type(None)):
            parallel_vert = None
        else:

            # First and last point of each parallel
            # In face XZ
            n_beams = n_drawers + 1
            beam_p0 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.zeros([n_beams,1])), axis=1)
            beam_p1 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.ones([n_beams,1]) * height), axis=1)
            
            # Adding the diagonals of the other face. Mirror orientation.
            beam_p0, beam_p1 = np.concatenate((beam_p0, beam_p1),axis=0), np.concatenate((beam_p1, beam_p0),axis=0)
            # Changing Y dimension
            beam_p0[n_beams:, 1] = width
            beam_p1[n_beams:, 1] = width

            # Updating number of diagonals 
            n_beams = len(beam_p0)
            
            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, parallel_vert)

        #############################################################################################
        # Horizontal down face
        #############################################################################################
        ## Diagonals    
        if isinstance(diagonal_bottom, type(None)):
            diagonal_bottom = None
        else:
          
            # First and last point of each diagonal
            # Left to right diagonals
            n_beams = n_drawers

            beam_p0 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.zeros([n_beams,1])), axis=1)
            beam_p1 = np.concatenate(((np.arange(n_beams) * length + length).reshape(-1,1),
                                       np.ones([n_beams,1]) * width, 
                                       np.zeros([n_beams,1])), axis=1)

            # Adding diagonals right to left
            beam_p0 = np.concatenate((beam_p0, 
                                      np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                       np.ones([n_beams,1]) * width, 
                                                       np.zeros([n_beams,1])), axis=1)),axis=0)
            beam_p1 = np.concatenate((beam_p1, 
                                      np.concatenate(((np.arange(n_beams) * length + length).reshape(-1,1),
                                                       np.zeros([n_beams,1]), 
                                                       np.zeros([n_beams,1])), axis=1)),axis=0)

            # Updating number of diagonals 
            n_beams = len(beam_p0)

            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, diagonal_bottom)

        ## Parallel
        if isinstance(parallel_bottom, type(None)):
            parallel_bottom = None
        else:

            # First and last point of each parallel
            n_beams = n_drawers + 1
            beam_p0 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.zeros([n_beams,1])), axis=1)
            beam_p1 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.ones([n_beams,1]) * width, 
                                       np.zeros([n_beams,1])), axis=1)
            
            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, parallel_bottom)

        #############################################################################################
        # Horizontal up face
        #############################################################################################
        ## Diagonals    
        if isinstance(diagonal_top, type(None)):
            diagonal_top = None
        else:
 
            # First and last point of each diagonal
            # Left to right diagonals
            n_beams = n_drawers

            beam_p0 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.ones([n_beams,1]) * height), axis=1)
            beam_p1 = np.concatenate(((np.arange(n_beams) * length + length).reshape(-1,1),
                                       np.ones([n_beams,1]) * width, 
                                       np.ones([n_beams,1]) * height), axis=1)

            # Adding diagonals right to left
            beam_p0 = np.concatenate((beam_p0, 
                                      np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                       np.ones([n_beams,1]) * width, 
                                                       np.ones([n_beams,1]) * height), axis=1)),axis=0)
            beam_p1 = np.concatenate((beam_p1, 
                                      np.concatenate(((np.arange(n_beams) * length + length).reshape(-1,1),
                                                       np.zeros([n_beams,1]), 
                                                       np.ones([n_beams,1]) * height), axis=1)),axis=0)

            # Updating number of diagonals 
            n_beams = len(beam_p0)

            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, diagonal_top)

        ## Parallel
        if isinstance(parallel_top, type(None)):
            parallel_top = None
        else:

            # First and last point of each parallel
            n_beams = n_drawers + 1
            beam_p0 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.zeros([n_beams,1]), 
                                       np.ones([n_beams,1]) * height), axis=1)
            beam_p1 = np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                       np.ones([n_beams,1]) * width, 
                                       np.ones([n_beams,1]) * height), axis=1)
            
            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, parallel_top)

        #############################################################################################
        # Inner faces
        #############################################################################################
        # Number of inner patterns
        inner_pat = 1 if parallel_inner is None else parallel_inner[0]+1

        ## Diagonals    
        if isinstance(diagonal_inner, type(None)):
            diagonal_inner = None
        else:
          
            # First and last point of each diagonal
            n_beams = n_drawers + 1

            beam_p0 = np.zeros((0,3))
            beam_p1 = np.zeros((0,3))
            for i in range(inner_pat):

                # Upper diagonals
                beam_p0 = np.concatenate((beam_p0,
                                          np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                           np.zeros([n_beams,1]) + width/inner_pat * i, 
                                                           np.zeros([n_beams,1])), axis=1)),axis=0)
                beam_p1 = np.concatenate((beam_p1,
                                          np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                           np.zeros([n_beams,1]) + width/inner_pat * (i+1), 
                                                           np.ones([n_beams,1]) * h_deck[0]), axis=1)),axis=0)

                # Adding downer diagonals
                beam_p0 = np.concatenate((beam_p0, 
                                        np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                        np.zeros([n_beams,1]) + width/inner_pat * i, 
                                                        np.ones([n_beams,1]) * h_deck[0]), axis=1)),axis=0)
                beam_p1 = np.concatenate((beam_p1, 
                                        np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                        np.zeros([n_beams,1]) + width/inner_pat * (i+1), 
                                                        np.zeros([n_beams,1])), axis=1)),axis=0)


            # Updating number of diagonals 
            n_beams = len(beam_p0)

            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, diagonal_inner)


        ## Parallel
        if parallel_inner is None:
            parallel_inner = None
        else:

            # First and last point of each parallel
            n_beams = n_drawers + 1

            beam_p0 = np.zeros((0,3))
            beam_p1 = np.zeros((0,3))
            for i in range(inner_pat-1):
                beam_p0 = np.concatenate((beam_p0,
                                          np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                           np.zeros([n_beams,1]) + width/inner_pat * (i+1), 
                                                           np.zeros([n_beams,1])), axis=1)),axis=0)
                beam_p1 = np.concatenate((beam_p1,
                                          np.concatenate(((np.arange(n_beams) * length).reshape(-1,1),
                                                           np.zeros([n_beams,1]) + width/inner_pat * (i+1), 
                                                           np.ones([n_beams,1]) * h_deck[0]), axis=1)),axis=0)
            
            # Update nodes
            node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem = TrussBridge.update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, parallel_inner[1:])

        ###########################################################################################################################
        # Remove repited nodes.
        node_coordinates, order = np.unique(node_coordinates, return_inverse=True, axis=0)
        beam_nodes = order[beam_nodes]

        ###########################################################################################################################
        # deck
        deck_points = np.asarray([[0, width/2, h_deck[0]+h_deck[1]/2],[length*n_drawers, width/2, h_deck[0]+h_deck[1]/2]])

        ###########################################################################################################################
        # Move Truss to the 0,0,0
        node_coordinates = node_coordinates - [n_drawers*length/2, width/2, height/2]
        deck_points = deck_points - [n_drawers*length/2, width/2, height/2]

        deck = [deck_points[0], deck_points[1], [width, h_deck[1]]]

        # TrussBridge constructor with the elements of BaileyTrusss
        super().__init__(node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem, deck, centre, orientation, density, cameras)
