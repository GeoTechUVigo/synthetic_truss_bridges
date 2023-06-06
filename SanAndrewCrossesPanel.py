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

def san_andrew_crosses_face(n_panels:int, diagonal:bool, parallel:bool, parallel_ends:bool):
    '''
    '''

    beam_p0_diag = np.zeros((0,3))
    beam_p1_diag = np.zeros((0,3))

    ## Diagonals    
    if diagonal:
        # Upper diagonals
        beam_p0_diag = np.concatenate((beam_p0_diag,
                                       np.concatenate(((np.arange(n_panels)/n_panels).reshape(-1,1),
                                                       np.zeros([n_panels,1]), 
                                                       np.zeros([n_panels,1])), axis=1)),axis=0)
        beam_p1_diag = np.concatenate((beam_p1_diag,
                                       np.concatenate((((np.arange(n_panels)+1)/n_panels).reshape(-1,1),
                                                        np.ones([n_panels,1]), 
                                                        np.zeros([n_panels,1])), axis=1)),axis=0)

        # Adding downer diagonals
        beam_p0_diag = np.concatenate((beam_p0_diag,
                                       np.concatenate(((np.arange(n_panels)/n_panels).reshape(-1,1),
                                                       np.ones([n_panels,1]), 
                                                       np.zeros([n_panels,1])), axis=1)),axis=0)
        beam_p1_diag = np.concatenate((beam_p1_diag,
                                       np.concatenate((((np.arange(n_panels)+1)/n_panels).reshape(-1,1),
                                                        np.zeros([n_panels,1]), 
                                                        np.zeros([n_panels,1])), axis=1)),axis=0)

    beam_p0_par = np.zeros((0,3))
    beam_p1_par = np.zeros((0,3))

    ## Parallel
    if parallel:
        beam_p0_par = np.concatenate((beam_p0_par,
                                      np.concatenate((((np.arange(n_panels-1)+1)/n_panels).reshape(-1,1),
                                                      np.zeros([n_panels-1,1]), 
                                                      np.zeros([n_panels-1,1])), axis=1)),axis=0)
        beam_p1_par = np.concatenate((beam_p1_par,
                                     np.concatenate((((np.arange(n_panels-1)+1)/n_panels).reshape(-1,1),
                                                     np.ones([n_panels-1,1]), 
                                                     np.zeros([n_panels-1,1])), axis=1)),axis=0)
        
    if parallel_ends:
        # first
        beam_p0_par = np.concatenate((np.array([0,0,0]).reshape(1,3),beam_p0_par), axis=0)
        beam_p1_par = np.concatenate((np.array([0,1,0]).reshape(1,3),beam_p0_par), axis=0)

        # last
        beam_p0_par = np.concatenate((beam_p0_par, np.array([1,0,0]).reshape(1,3)), axis=0)
        beam_p1_par = np.concatenate((beam_p0_par, np.array([1,1,0]).reshape(1,3)), axis=0)

    return beam_p0_diag, beam_p1_diag, beam_p0_par, beam_p1_par