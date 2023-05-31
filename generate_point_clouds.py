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

import numpy as np
from BaileyTruss import BaileyTruss
from BrownTruss import BrownTruss
import pathlib
import pandas as pd
import open3d as o3d

# ============================================================================

# Parameters

PATH_OUT = "data/occlusions_and_nodes"
FILE_TYPE = ".laz"

# TRUSS PARAMETERS
N_POINT_CLOUDS = 100
DRAWERS = [4,10]
HEIGH = [3,5]
LENGHT = [3,5]
WIDTH = [2,5]
INNER_PATERNS = [1,3]
DENSITY = 1000

# ORIENTATION
YAW=np.pi
PITCH=np.arctan(20/100)
ROLL=np.arctan(5/100)

# OTHER PARAMETERS
SEED = 100
DECIMALS = 0.01
N_DIGITS = len(str(N_POINT_CLOUDS))
DISTANCE_NODES = 0.2 # Dist to node to classify that points as node

# LILDAR
# Angles
LIDAR_STEP_DEG = 5
LIDAR_ANGLE_DEG = 150
LIDAR_ANGLE_DEG_DECK = 360
# Number of scans by position of the LiDAR in the bridge
N_CAMARA_DECK = [2,4]
N_CAMARA_DOWN = [0,5]
N_CAMARA_LAT = [0,5] # It is done in both sides
MIN_CAM = 3
# Positions
CAM_DIST_DECK = [0.5, 1.5] # distance Z to the deck
CAM_DIST_DOWN = [-3, -10] # distance Z to the down face.
CAM_DIST_LAT_Y = [3, 10] # distance to the lateral face in Y. Both sides
CAM_DIST_LAT_Z = [2, -10] # distance to the centre in Z
SIGMA = [1.0, 0.2, 0] # Standar deviation in XYZ. Used in the dimensions in which there are not random choice

# ============================================================================
np.random.seed(SEED)
PATH_OUT = pathlib.Path(PATH_OUT)
if not PATH_OUT.is_dir(): raise ValueError("PATH_OUT {} does not exist.".format(str(PATH_OUT)))

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
    centre = np.asarray([np.random.uniform(0,100), np.random.uniform(0,100), np.random.uniform(0,100)])
    orientation = np.asarray([np.random.uniform(-YAW,YAW), np.random.uniform(-PITCH,PITCH), np.random.uniform(-ROLL,ROLL)])

    # Dimensions
    n_drawers = np.random.randint(DRAWERS[0], DRAWERS[1])
    height = np.random.random() * (HEIGH[1] - HEIGH[0]) + HEIGH[0]
    length = np.random.random() * (LENGHT[1] - LENGHT[0]) + LENGHT[0]
    width = np.random.random() * (WIDTH[1] - WIDTH[0]) + WIDTH[0]

    # Deck
    if np.random.random() < 0.7:
        deck_position = np.random.random()*height/2
    else:
        deck_position = height

    h_deck = [deck_position, np.random.random()*0.2+0.1]

    # Elements
    chords = [np.random.choice(profiles_chord), 0 + np.random.randint(0,4)*(np.pi/2), 1]
    diagonals_vert = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if np.random.random() < 0.7 else None
    parallels_vert = [np.random.choice(profiles_vert), np.random.randint(0,4)*(np.pi/2), 2] if np.random.random() < 1 else None
    diagonals_bottom = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if np.random.random() < 0.7 else None
    parallels_bottom = [np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2] if np.random.random() < 0.7 else None
    diagonals_top = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if np.random.random() < 0.7 and deck_position!=height else None
    parallels_top = [np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2] if np.random.random() < 0.7 and deck_position!=height else None
    parallels_inner = [np.random.randint(np.clip(1,INNER_PATERNS[0], np.inf), INNER_PATERNS[1]-1), np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2] if np.random.random() < 0.7 and deck_position>1 else None
    diagonals_inner = [np.random.choice(profiles_inner), np.random.randint(0,4)*(np.pi/2), 3] if (np.random.random() < 0.7 and deck_position>1) or parallels_inner is not None else None

    #=================================================================================================================================================
    # Positions of the LIDAR. their are calcualted consider the point centre in (0,0,0).
    # Number of cams
    n_cameras_deck = np.random.randint(N_CAMARA_DECK[0], N_CAMARA_DECK[1]) if deck_position != height else 0
    n_cameras_down = np.random.randint(N_CAMARA_DOWN[0], N_CAMARA_DOWN[1])
    n_cameras_lat_pos = np.random.randint(N_CAMARA_LAT[0], N_CAMARA_LAT[1])
    n_cameras_lat_neg = np.random.randint(N_CAMARA_LAT[0], N_CAMARA_LAT[1])

    while n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < MIN_CAM:
        if N_CAMARA_LAT[1] > n_cameras_lat_pos:
            n_cameras_lat_pos = np.random.randint(n_cameras_lat_pos, N_CAMARA_LAT[1]) 
        if n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < MIN_CAM: break
        if N_CAMARA_LAT[1] > n_cameras_lat_neg:
            n_cameras_lat_neg = np.random.randint(n_cameras_lat_neg, N_CAMARA_LAT[1]) 
        if n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < MIN_CAM: break
        if N_CAMARA_DOWN[1] > n_cameras_down:
            n_cameras_down = np.random.randint(n_cameras_down, N_CAMARA_DOWN[1]) 
        if n_cameras_deck+n_cameras_down+n_cameras_lat_pos+n_cameras_lat_neg < MIN_CAM: break
        if N_CAMARA_DECK[1] > n_cameras_deck:
            n_cameras_deck = np.random.randint(n_cameras_deck, N_CAMARA_DECK[1]) if deck_position != height else 0

    # DECK. 4 cameras_deck por position to do 360.
    angle = LIDAR_ANGLE_DEG_DECK/4
    px = int(angle*LIDAR_STEP_DEG)
    # Initialising
    cameras_deck = np.zeros(n_cameras_deck*4, dtype='object')
    for j in range(n_cameras_deck):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_drawers/2 + (j+1)/(n_cameras_deck+1)* length*n_drawers + np.random.normal(scale=SIGMA[0])
        position[1] = 0 + np.random.normal(scale=SIGMA[1])
        position[2] = -height/2+deck_position + np.random.random() * (CAM_DIST_DECK[1] - CAM_DIST_DECK[0]) + CAM_DIST_DECK[0]

        # pointing +X and -X.
        cameras_deck[j*4] = {'fov_deg': angle, 'center':position + [1,0,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}
        cameras_deck[j*4+1] = {'fov_deg': angle, 'center':position + [-1,0,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

        # pointing +Y and -Y
        cameras_deck[j*4+2] = {'fov_deg': angle, 'center':position + [0,1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}
        cameras_deck[j*4+3] = {'fov_deg': angle, 'center':position + [0,-1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    angle = LIDAR_ANGLE_DEG
    px = int(angle*LIDAR_STEP_DEG)

    #DOWN
    # Initialising
    cameras_down = np.zeros(n_cameras_down, dtype='object')
    for j in range(n_cameras_down):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_drawers/2 + (j+1)/(n_cameras_down+1)* length*n_drawers + np.random.normal(scale=SIGMA[0])
        position[1] = 0 + np.random.normal(scale=SIGMA[1])
        position[2] = -height/2 + np.random.random() * (CAM_DIST_DOWN[1] - CAM_DIST_DOWN[0]) + CAM_DIST_DOWN[0]

        # camera pointing +Z
        cameras_down[j] = {'fov_deg': angle, 'center':position + [0,0,1], 'eye':position, 'up':[1,0,0], 'width_px':px, 'height_px':px}

    #LATERAL +Y
    # Initialising
    cameras_lateral_pos = np.zeros(n_cameras_lat_pos, dtype='object')
    for j in range(n_cameras_lat_pos):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_drawers/2 + (j+1)/(n_cameras_lat_pos+1)* length*n_drawers + np.random.normal(scale=SIGMA[0])
        position[1] = width/2 + np.random.random() * (CAM_DIST_LAT_Y[1] - CAM_DIST_LAT_Y[0]) + CAM_DIST_LAT_Y[0]
        position[2] = np.random.random() * (CAM_DIST_LAT_Z[1] - CAM_DIST_LAT_Z[0]) + CAM_DIST_LAT_Z[0]

        # camera pointing -Y
        cameras_lateral_pos[j] = {'fov_deg': angle, 'center':position + [0,-1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    #LATERAL -Y
    # Initialising
    cameras_lateral_neg = np.zeros(n_cameras_lat_neg, dtype='object')
    for j in range(n_cameras_lat_neg):
        # Spacing the positions.
        position = np.zeros(3)
        position[0] = -length*n_drawers/2 + (j+1)/(n_cameras_lat_neg+1)* length*n_drawers + np.random.normal(scale=SIGMA[0])
        position[1] = -width/2 - np.random.random() * (CAM_DIST_LAT_Y[1] - CAM_DIST_LAT_Y[0]) + CAM_DIST_LAT_Y[0]
        position[2] = np.random.random() * (CAM_DIST_LAT_Z[1] - CAM_DIST_LAT_Z[0]) + CAM_DIST_LAT_Z[0]

        # camera pointing +Y
        cameras_lateral_neg[j] = {'fov_deg': angle, 'center':position + [0,1,0], 'eye':position, 'up':[0,0,1], 'width_px':px, 'height_px':px}

    cameras = np.concatenate((cameras_deck, cameras_down, cameras_lateral_pos, cameras_lateral_neg))

    #=================================================================================================================================================
    
    my_bridge = BrownTruss(n_drawers=n_drawers, height=height, length=length, width=width, h_deck=h_deck, chord=chords, 
                            diagonal_vert=diagonals_vert, parallel_vert=parallels_vert,
                            diagonal_bottom=diagonals_bottom, parallel_bottom=parallels_bottom,
                            diagonal_top=diagonals_top, parallel_top=parallels_top,
                            diagonal_inner=diagonals_inner, parallel_inner=parallels_inner,
                            centre=centre, orientation=orientation,
                            density=DENSITY, cameras=cameras
                            )

    # my_bridge.show_pc()

    # Visualization
    # mesh = my_bridge.mesh()
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(my_bridge.node_coordinates)
    # o3d.visualization.draw([mesh])
    # coordinates = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # point_cloud = my_bridge.point_cloud()
    # o3d.visualization.draw_geometries([point_cloud, coordinates])

    file_name = my_bridge.name + "_" + str(i).zfill(N_DIGITS) + FILE_TYPE

    file_path = PATH_OUT.joinpath(file_name)

    my_bridge.save_las(path=file_path, scale=DECIMALS, distance_nodes=DISTANCE_NODES)