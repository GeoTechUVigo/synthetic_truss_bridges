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

# ============================================================================
"""
Generate and save Truss bridges in LAZ format with semantic segmentation
in classfication attribute and instance segmentation in user data attribute.
"""
# ============================================================================

import random
import numpy as np
from BaileyTruss import BaileyTruss
from BrownTruss import BrownTruss
import pathlib
import pandas as pd
import open3d as o3d

# ============================================================================

# Parameters

path_out = "synthetic_data"
extension = ".las"

N_POINT_CLOUDS = 100
DRAWERS = [4,10]
HEIGH = [3,5]
LENGHT = [3,5]
WIDTH = [2,5]
DENSITY = 1000
DECIMALS = 0.01
YAW=np.pi
PITCH=np.arctan(20/100)
ROLL=np.arctan(5/100)
SEED = 100
N_DIGITS = len(str(N_POINT_CLOUDS))
DISTANCE_NODES = 0.2

path_out = pathlib.Path(path_out)
random.seed(a=SEED)

# list profiles by type of beam
profile_path = "standarized_profiles_formatted"
profiles_names = np.ndarray(0, dtype='<U16')
profile_path = pathlib.Path(profile_path)
for file in profile_path.iterdir():
    profile_data_frame = pd.read_csv(file)
    profiles_names =  np.append(profiles_names, np.asarray(profile_data_frame['Designation'], dtype=profiles_names.dtype))

# Type of profile and main dimension
profile_type = np.ndarray(0, dtype='<U3')
profile_dim = np.ndarray(0, dtype='int')
for type in np.char.split(profiles_names,('-')):
    profile_type = np.append(profile_type,type[0])
    profile_dim = np.append(profile_dim, int(type[1].split('x')[0]))

# list profiles by type of beam
profiles_chord = profiles_names[np.logical_and(np.in1d(profile_type, ['IPN','UB', 'UBP', 'UC', 'U', 'UPE', 'UPN', 'CH', 'HD', 'HL', 'HLZ', 'HP']), profile_dim >= 300)]
profiles_vert = profiles_names[np.logical_and(np.in1d(profile_type, ['IPN', 'HD', 'HL', 'HLZ', 'HP']), np.logical_and(profile_dim >= 200, profile_dim <= 300))]
profiles_parallel = profiles_names[np.logical_and(np.in1d(profile_type, ['IPN', 'L']), np.logical_and(profile_dim >= 200, profile_dim <= 300))]
profiles_diagonals = profiles_names[np.logical_and(np.in1d(profile_type, ['IPN', 'L', 'HD', 'HL', 'HLZ', 'HP']), profile_dim <= 200)]
profiles_inner = profiles_names[np.logical_and(np.in1d(profile_type, ['L']), profile_dim <= 200)]

for i in range(N_POINT_CLOUDS):

    # Centre and orientation
    centre = [0,0,0]
    # centre = [random.uniform(0,100), random.uniform(0,100), random.uniform(0,100)]
    orientation = [random.uniform(-YAW,YAW), random.uniform(-PITCH,PITCH), random.uniform(-ROLL,ROLL)]

    # Dimensions
    n_drawers = random.randint(DRAWERS[0], DRAWERS[1])
    height = random.random() * (HEIGH[1] - HEIGH[0]) + HEIGH[0]
    length = random.random() * (LENGHT[1] - LENGHT[0]) + LENGHT[0]
    width = random.random() * (WIDTH[1] - WIDTH[0]) + WIDTH[0]

    # Deck
    if random.random() < 0.7:
        deck_position = random.random()*height/2
    else:
        deck_position = height

    h_deck = [deck_position,random.random()*0.2+0.1]

    # Elements
    chords = [random.choice(profiles_chord), 0 + random.randint(0,4)*(np.pi/2), 1]
    diagonals_vert = [random.choice(profiles_diagonals), random.randint(0,4)*(np.pi/2), 3] if random.random() < 0.7 else None
    parallels_vert = [random.choice(profiles_vert), random.randint(0,4)*(np.pi/2), 2] if random.random() < 1 else None
    diagonals_bottom = [random.choice(profiles_diagonals), random.randint(0,4)*(np.pi/2), 3] if random.random() < 0.7 else None
    parallels_bottom = [random.choice(profiles_parallel), random.randint(0,4)*(np.pi/2), 2] if random.random() < 0.7 else None
    diagonals_top = [random.choice(profiles_diagonals), random.randint(0,4)*(np.pi/2), 3] if random.random() < 0.7 and deck_position!=height else None
    parallels_top = [random.choice(profiles_parallel), random.randint(0,4)*(np.pi/2), 2] if random.random() < 0.7 and deck_position!=height else None
    diagonals_inner = [random.choice(profiles_inner), random.randint(0,4)*(np.pi/2), 3] if random.random() < 0.7 and deck_position>1 else None

    my_bridge = BrownTruss(n_drawers=n_drawers, height=height, length=length, width=width, h_deck=h_deck, chord=chords, 
                            diagonal_vert=diagonals_vert, parallel_vert=parallels_vert,
                            diagonal_bottom=diagonals_bottom, parallel_bottom=parallels_bottom,
                            diagonal_top=diagonals_top, parallel_top=parallels_top,
                            diagonal_inner=diagonals_inner,
                            centre=centre, orientation=orientation,
                            density=DENSITY
                            )

    # oclusions
    mesh = my_bridge.occlusions(camera = 0)

    # Visualization
    # mesh = my_bridge.mesh()
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(my_bridge.node_coordinates)
    # o3d.visualization.draw([mesh])
    # coordinates = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # point_cloud = my_bridge.point_cloud()
    # o3d.visualization.draw_geometries([point_cloud, coordinates])

    file_name = my_bridge.name + "_" + str(i).zfill(N_DIGITS) + extension

    file_path = path_out.joinpath(file_name)

    my_bridge.save_las(path=file_path, scale=DECIMALS, distance_nodes=DISTANCE_NODES)