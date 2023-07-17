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
import json
from camera_positions import camera_positions

# ============================================================================

# Parameters
FILE_TYPE = ".las"
PATH_OUT='data'
PATH_OUT_UNIFORM = "data/uniform"
PATH_OUT_OCCLUSIONS = "data/occlusions"

PATH_GRAPH = "data/several_inner_diag/graphs"
PATH_TRANSFORM = "data/several_inner_diag/transform"
PATH_DIST_NODES = ""
SAVE_PARAMETRES = True
SAVE_GRAPH = None
SAVE_TRANSFORM = None
SAVE_DISTANCES = None

# TRUSS PARAMETERS
N_POINT_CLOUDS = 250
DRAWERS = [4,10]
HEIGH = [3,5]
LENGHT = [3,5]
WIDTH = [2,5]
INNER_PATERNS = [0,0]
DIAG_PANEL = [1,1]
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
N_CAMERA_DECK = [2,3]
N_CAMERA_DOWN = [2,4]
N_CAMERA_LAT = [2,4] # It is done in both sides
MIN_CAM = 6
# Positions
CAM_DIST_DECK = [0.5, 1.5] # distance Z to the deck
CAM_DIST_DOWN = [-3, -10] # distance Z to the down face.
CAM_DIST_LAT_Y = [3, 10] # distance to the lateral face in Y. Both sides
CAM_DIST_LAT_Z = [2, -10] # distance to the centre in Z
SIGMA = [1.0, 0.2, 0] # Standar deviation in XYZ. Used in the dimensions in which there are not random choice

# SECTIONS
CHORD = {'profile': ['IPN','UB', 'UBP', 'UC', 'U', 'UPE', 'UPN', 'CH', 'HD', 'HL', 'HLZ', 'HP'], 'min': 300, 'max': np.inf}
VERT = {'profile': ['IPN', 'HD', 'HL', 'HLZ', 'HP'], 'min': 200, 'max': 300}
PARALLEL = {'profile':['IPN', 'L'], 'min': 200, 'max': 300}
DIAGONALS = {'profile':['IPN', 'L', 'HD', 'HL', 'HLZ', 'HP'],  'min': 0, 'max': 200}
INNER = {'profile':['L'],  'min': 0, 'max': 200}
# ============================================================================
np.random.seed(SEED)

PATH_OUT = pathlib.Path(PATH_OUT)
PATH_OUT_UNIFORM = pathlib.Path(PATH_OUT_UNIFORM)
PATH_OUT_OCCLUSIONS = pathlib.Path(PATH_OUT_OCCLUSIONS)
PATH_GRAPH = pathlib.Path(PATH_GRAPH)
PATH_TRANSFORM = pathlib.Path(PATH_TRANSFORM)
PATH_DIST_NODES = pathlib.Path(PATH_DIST_NODES)
if not PATH_OUT.is_dir(): raise ValueError("PATH_OUT {} does not exist.".format(str(PATH_OUT)))
if not PATH_OUT_UNIFORM.is_dir(): raise ValueError("PATH_OUT_UNIFORM {} does not exist.".format(str(PATH_OUT_UNIFORM)))
if not PATH_OUT_OCCLUSIONS.is_dir(): raise ValueError("PATH_OUT_OCCLUSIONS {} does not exist.".format(str(PATH_OUT_OCCLUSIONS)))
if not PATH_GRAPH.is_dir() and SAVE_GRAPH: raise ValueError("PATH_GRAPH {} does not exist.".format(str(PATH_GRAPH)))
if not PATH_TRANSFORM.is_dir() and SAVE_TRANSFORM: raise ValueError("PATH_TRANSFORM {} does not exist.".format(str(PATH_TRANSFORM)))
if not PATH_DIST_NODES.is_dir() and SAVE_DISTANCES: raise ValueError("PATH_DIST_NODES {} does not exist.".format(str(PATH_DIST_NODES)))

if SAVE_PARAMETRES:
    parametres = dict()
    parametres['DRAWERS'] = DRAWERS
    parametres['HEIGH'] = HEIGH
    parametres['LENGHT'] = LENGHT
    parametres['WIDTH'] = WIDTH
    parametres['INNER_PATERNS'] = INNER_PATERNS
    parametres['DIAG_PANEL'] = DIAG_PANEL
    parametres['DENSITY'] = DENSITY
    parametres['YAW'] = YAW
    parametres['PITCH'] = PITCH
    parametres['ROLL'] = ROLL
    parametres['SEED'] = SEED
    parametres['DECIMALS'] = DECIMALS
    parametres['LIDAR_STEP_DEG'] = LIDAR_STEP_DEG
    parametres['LIDAR_ANGLE_DEG'] = LIDAR_ANGLE_DEG
    parametres['LIDAR_ANGLE_DEG_DECK'] = LIDAR_ANGLE_DEG_DECK
    parametres['N_CAMERA_DECK'] = N_CAMERA_DECK
    parametres['N_CAMERA_DOWN'] = N_CAMERA_DOWN
    parametres['N_CAMERA_LAT'] = N_CAMERA_LAT
    parametres['MIN_CAM'] = MIN_CAM
    parametres['CAM_DIST_DECK'] = CAM_DIST_DECK
    parametres['CAM_DIST_DOWN'] = CAM_DIST_DOWN
    parametres['CAM_DIST_LAT_Y'] = CAM_DIST_LAT_Y
    parametres['CAM_DIST_LAT_Z'] = CAM_DIST_LAT_Z
    parametres['SIGMA'] = SIGMA
    parametres['CHORD'] = CHORD
    parametres['VERT'] = VERT
    parametres['PARALLEL'] = PARALLEL
    parametres['DIAGONALS'] = DIAGONALS
    parametres['INNER'] = INNER

    with open(str(PATH_OUT.joinpath('parametres.json')), 'w') as f:
        my_json = json.dumps(parametres, indent=4)
        f.write(my_json)

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
profiles_chord = profiles_names[np.logical_and(np.in1d(profile_type, CHORD['profile']), np.logical_and(profile_dim >= CHORD['min'], profile_dim <= CHORD['max']))]
profiles_vert = profiles_names[np.logical_and(np.in1d(profile_type, VERT['profile']), np.logical_and(profile_dim >= VERT['min'], profile_dim <= VERT['max']))]
profiles_parallel = profiles_names[np.logical_and(np.in1d(profile_type, PARALLEL['profile']), np.logical_and(profile_dim >= PARALLEL['min'], profile_dim <= PARALLEL['max']))]
profiles_diagonals = profiles_names[np.logical_and(np.in1d(profile_type, DIAGONALS['profile']), np.logical_and(profile_dim >= DIAGONALS['min'], profile_dim <= DIAGONALS['max']))]
profiles_inner = profiles_names[np.logical_and(np.in1d(profile_type, INNER['profile']), np.logical_and(profile_dim >= INNER['min'], profile_dim <= INNER['max']))]

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
    under_deck_elements = False
    if np.random.random() < 0.7:
        deck_position = 0.5 + np.random.random()*height/2
        under_deck_elements = True
    elif np.random.random() < 0.5:
        deck_position = 0
    else:
        deck_position = height

    h_deck = [deck_position, np.random.random()*0.2+0.1]

    # Elements
    chords = [np.random.choice(profiles_chord), 0 + np.random.randint(0,4)*(np.pi/2), 1]
    diagonals_vert = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if np.random.random() < 0.7 else None
    parallels_vert = [np.random.choice(profiles_vert), np.random.randint(0,4)*(np.pi/2), 2]
    diagonals_bottom = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if under_deck_elements and np.random.random() < 0.7 else None
    parallels_bottom = [np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2]
    parallels_deck = None #[np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2] if under_deck_elements else None
    diagonals_top = [np.random.choice(profiles_diagonals), np.random.randint(0,4)*(np.pi/2), 3] if np.random.random() < 0.7 and deck_position!=height else None
    parallels_top = [np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2]
    diagonals_inner = [np.random.choice(profiles_inner), np.random.randint(0,4)*(np.pi/2), 3] if (np.random.random() < 0.7 and deck_position>1) or parallels_inner is not None else None
    if INNER_PATERNS[1] == 0:
        parallels_inner = None
    else:
        parallels_inner = [np.random.randint(np.clip(1,INNER_PATERNS[0], np.inf), INNER_PATERNS[1]+1), np.random.choice(profiles_parallel), np.random.randint(0,4)*(np.pi/2), 2] if np.random.random() < 0.7 and deck_position>1 else None

    n_diag = np.random.randint(DIAG_PANEL[0], DIAG_PANEL[1]+1)

    #=================================================================================================================================================
    # Positions of the LIDAR. their are calcualted consider the point centre in (0,0,0).
    cameras = camera_positions(N_CAMERA_DECK, N_CAMERA_DOWN, N_CAMERA_LAT, LIDAR_ANGLE_DEG, LIDAR_ANGLE_DEG_DECK, LIDAR_STEP_DEG, 
                     CAM_DIST_DECK, CAM_DIST_DOWN, CAM_DIST_LAT_Y, CAM_DIST_LAT_Z, 
                     MIN_CAM, deck_position, height, length, width, n_drawers, SIGMA)

    #=================================================================================================================================================
    # Typology
    if i<N_POINT_CLOUDS/2:
        BridgeType = BaileyTruss
    else:
        BridgeType = BrownTruss

    # occlusions
    my_bridge = BridgeType(n_drawers=n_drawers, height=height, length=length, width=width, h_deck=h_deck, chord=chords, 
                            diagonal_vert=diagonals_vert, parallel_vert=parallels_vert,
                            diagonal_bottom=diagonals_bottom, parallel_bottom=parallels_bottom, parallel_deck = parallels_deck,
                            diagonal_top=diagonals_top, parallel_top=parallels_top,
                            diagonal_inner=diagonals_inner, parallel_inner=parallels_inner,
                            centre=centre, orientation=orientation,
                            density=DENSITY, cameras=cameras
                            )
    file_name = pathlib.Path(my_bridge.name + "_" + str(i).zfill(N_DIGITS))
    file_path = PATH_OUT_OCCLUSIONS.joinpath(file_name).with_suffix(FILE_TYPE)
    my_bridge.save_las(path=file_path, scale=DECIMALS)
    
    # non occlusions
    my_bridge = BridgeType(n_drawers=n_drawers, height=height, length=length, width=width, h_deck=h_deck, chord=chords, 
                            diagonal_vert=diagonals_vert, parallel_vert=parallels_vert,
                            diagonal_bottom=diagonals_bottom, parallel_bottom=parallels_bottom, parallel_deck = parallels_deck,
                            diagonal_top=diagonals_top, parallel_top=parallels_top,
                            diagonal_inner=diagonals_inner, parallel_inner=parallels_inner,
                            centre=centre, orientation=orientation,
                            density=DENSITY
                            )
    file_name = pathlib.Path(my_bridge.name + "_" + str(i).zfill(N_DIGITS))
    file_path = PATH_OUT_UNIFORM.joinpath(file_name).with_suffix(FILE_TYPE)
    my_bridge.save_las(path=file_path, scale=DECIMALS)

    # my_bridge.show_pc()

    # Visualization
    # mesh = my_bridge.mesh()
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(my_bridge.node_coordinates)
    # o3d.visualization.draw([mesh])
    # coordinates = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # point_cloud = my_bridge.point_cloud()
    # o3d.visualization.draw_geometries([point_cloud, coordinates])

    # Save transformation matrix
    if SAVE_TRANSFORM:
        file_path = PATH_TRANSFORM.joinpath(file_name).with_suffix('.txt')
        my_bridge.write_transformation(file_path)

    # Save graph
    if SAVE_GRAPH:
        file_path = PATH_GRAPH.joinpath(file_name).with_suffix('.json')
        my_bridge.graph.write(file_path)

    # Save distance between nodes
    if SAVE_DISTANCES:
        file_path = PATH_DIST_NODES.joinpath(file_name).with_suffix('.npy')

        dist_nodes = np.repeat(np.expand_dims(my_bridge.node_coordinates, axis=0), len(my_bridge.node_coordinates), axis=0)
        dist_nodes = np.linalg.norm(dist_nodes - np.transpose(dist_nodes, axes=[1,0,2]), axis=2)
        np.save(str(file_path), dist_nodes)