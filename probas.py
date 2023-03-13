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

import random
import numpy as np
import open3d as o3d
from BaileyTruss import BaileyTruss
from BrownTruss import BrownTruss
from TrussBridge import TrussBridge
import copy
import pandas as pd
import pathlib

FILE_PATH = "/home/lamas/Documentos/GitHub/SAFEWAY_truss_bridge_deep_learning/Images/LAS/point_cloud.las"


chord = ["IPN-600", np.pi/2, 1]
diagonals_vert = None
parallels_vert = None
diagonals_bottom =None
parallels_bottom = None
diagonals_top = None
parallels_top = None
diagonals_inner = None

chord = ["IPN-600", 0, 1]
diagonals_vert = ["U-65x42x5.5", 0, 3]
parallels_vert = ["HD-260x114", 0, 2]
# diagonals_bottom = ["U-65x42x5.5", np.pi/2, 3]
parallels_bottom = ["HD-260x114", np.pi/2, 2]
# diagonals_top = ["HD-260x114", np.pi/2, 3]
# parallels_top = ["HD-400x382", np.pi/2, 2]
diagonals_inner = ["UPE-80", np.pi/2, 3]

my_bridge = BrownTruss(n_drawers=5, height=4, length=3, width=2, h_deck=[2.5, 0.3], 
                       chord=chord, diagonal_vert=diagonals_vert, parallel_vert=parallels_vert, 
                       diagonal_bottom= diagonals_bottom, parallel_bottom=parallels_bottom,
                       diagonal_top = diagonals_top, parallel_top = parallels_top,
                       diagonal_inner =diagonals_inner, centre= [0,0,0], orientation=[0,0,0],
                       density=1000
                       )

my_bridge.show_pc()