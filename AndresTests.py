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
import open3d as o3d
from Beam import Beam
import copy

# beam = Beam(np.asarray([10, 0, 0]), np.asarray([0, 0, 0]), "T-20x20", 0.0)
beam = Beam(np.asarray([10, 0, 0]), np.asarray([0, 0, 0]), "HSS-40x20x2", 0.0)


coordinates = o3d.geometry.TriangleMesh.create_coordinate_frame()
rotatedCoord = copy.deepcopy(coordinates).transform(beam.rotM)
o3d.visualization.draw_geometries([coordinates, beam.mesh,rotatedCoord])

# "Opening" of the profile is always on the X axis
# After orienting the profile, its local x axis goes from 0 to -1 depending on the target vector inclination
# From increments of 90 degrees in the rotation angle [0,90,180,270], the opening points towards:
# RightFace Diagonal or chord: Downwards, Inside, Upwards, Outside. -> Parallel to axis: Neg Z, Pos Y, Pos Z, Neg Y
# LeftFace Diagonal: Downwards, Outside, Upwards, Inside.  -> Parallel to a given axis: Neg Z, Pos Y, Pos Z, Neg Y
# Top Brace: Downwards/Inside, Aguas abajo, Upwards/Outside, Aguas arriba -> Aguas are not parallel to a given axis
# Bottom Brace: Downwards/Outside, Aguas abajo, Upwards/Inside, Aguas arriba -> Aguas are not parallel to a given axis
# Vertical members: Aguas arriba, Inside, Aguas abajo, Outside
# Use negative orientation angle for mirror effect between faces.

beamXAxis = beam.rotM[0:3, 0]
beamYAxis = beam.rotM[0:3, 1]
beamZAxis = beam.rotM[0:3, 2]

isOpeningXAxis = np.allclose(np.asarray([1,0,0]),beamXAxis,atol=.01,rtol=.01)
isOpeningYAxis = np.allclose(np.asarray([0,1,0]),beamXAxis,atol=.01,rtol=.01)
isOpeningZAxis = np.allclose(np.asarray([0,0,1]),beamXAxis,atol=.01,rtol=.01)
isOpeningNegXAxis = np.allclose(np.asarray([-1,0,0]),beamXAxis,atol=.01,rtol=.01)
isOpeningNegYAxis = np.allclose(np.asarray([0,-1,0]),beamXAxis,atol=.01,rtol=.01)
isOpeningNegZAxis = np.allclose(np.asarray([0,0,-1]),beamXAxis,atol=.01,rtol=.01)

if not np.any([isOpeningXAxis,isOpeningYAxis,isOpeningZAxis,isOpeningNegXAxis,isOpeningNegYAxis,isOpeningNegZAxis]):
    print("What")







