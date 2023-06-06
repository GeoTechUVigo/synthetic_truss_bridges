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

import pathlib
import open3d as o3d
import laspy
import numpy as np

folder_path_heuristic = '/home/lamas/Documentos/GitHub/SAFEWAY_truss_bridge_deep_learning/instance_segmentation/JSNet/data/test_heuristic'
folder_path_heuristic_errors = '/home/lamas/Documentos/GitHub/SAFEWAY_truss_bridge_deep_learning/instance_segmentation/JSNet/data/test_heuristic/errors'
folder_path_dl = '/home/lamas/Documentos/GitHub/SAFEWAY_truss_bridge_deep_learning/instance_segmentation/JSNet/data/test_dl'
folder_path_dl_errors = '/home/lamas/Documentos/GitHub/SAFEWAY_truss_bridge_deep_learning/instance_segmentation/JSNet/data/test_dl/errors'
suffix = '.las'

folder_path_heuristic = pathlib.Path(folder_path_heuristic)
folder_path_heuristic_errors = pathlib.Path(folder_path_heuristic_errors)
folder_path_dl = pathlib.Path(folder_path_dl)
folder_path_dl_errors = pathlib.Path(folder_path_dl_errors)

# Plot errors
def plot_errors(point_cloud_path):
    # read point cloud
    point_cloud = laspy.read(str(point_cloud_path))

    pcd_sem = o3d.geometry.PointCloud()
    pcd_sem.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    #=========================================================
    # semantic colours
    colours = np.zeros((len(point_cloud),3))

    colours[point_cloud.classification == 0] = np.array([0,1,0])
    colours[point_cloud.classification == 1] = np.array([1,0,0])
    colours[point_cloud.classification == 2] = np.array([0,0,0])

    pcd_sem.colors = o3d.utility.Vector3dVector(colours)

    #=========================================================
    # instance colours
    pcd_ins = o3d.geometry.PointCloud()
    pcd_ins.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    colours = np.zeros((len(point_cloud),3))

    colours[point_cloud.user_data == 0] = np.array([0,1,0])
    colours[point_cloud.user_data == 1] = np.array([1,0,0])
    colours[point_cloud.user_data == 2] = np.array([0,0,0])

    pcd_ins.colors = o3d.utility.Vector3dVector(colours)

    o3d.visualization.draw([pcd_ins], title=str(point_cloud_path))
    o3d.visualization.draw([pcd_sem], title=str(point_cloud_path))


# Plot segmentation
def plot_segmentation(point_cloud_path):
    # read point cloud
    point_cloud = laspy.read(str(point_cloud_path))

    pcd_sem = o3d.geometry.PointCloud()
    pcd_sem.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    #=========================================================
    # semantic colours
    colours = np.zeros((len(point_cloud),3))

    colours[point_cloud.classification == 0] = np.array([0.5,0.5,0.5])
    colours[point_cloud.classification == 1] = np.array([0,0,1])
    colours[point_cloud.classification == 2] = np.array([0,1,0])
    colours[point_cloud.classification == 3] = np.array([1,0,0])

    pcd_sem.colors = o3d.utility.Vector3dVector(colours)

    #=========================================================
    # instance colours
    pcd_ins = o3d.geometry.PointCloud()
    pcd_ins.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    inst = np.unique(point_cloud.user_data)
    colours_group = np.random.rand(len(inst),3)

    colours = np.zeros((len(point_cloud),3))

    for i in inst:
        colours[point_cloud.user_data==i,:] = colours_group[i]

    pcd_ins.colors = o3d.utility.Vector3dVector(colours)

    o3d.visualization.draw([pcd_ins], title=str(point_cloud_path))
    o3d.visualization.draw([pcd_sem], title=str(point_cloud_path))


# Plot segmentation heuristic
def plot_segmentation_heuristic(point_cloud_path):
    # read point cloud
    point_cloud = laspy.read(str(point_cloud_path))

    pcd_sem = o3d.geometry.PointCloud()
    pcd_sem.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    # Classification labels heurist method.
    # Vertical face Lateral -> 1
    # Vertical face Vertical -> 2
    # Chord -> 3
    # Horizontal face Lateral -> 4
    # Horizontal face Vertical -> 5
    # Inner face Lateral -> 6
    # Inner face Horizontal -> 7

    # Adapt labels to real labels
    sem_pred = np.zeros(point_cloud.classification.shape)
    sem_pred[point_cloud.classification==1] = 3
    sem_pred[point_cloud.classification==2] = 2
    sem_pred[point_cloud.classification==3] = 1
    sem_pred[point_cloud.classification==4] = 3
    sem_pred[point_cloud.classification==5] = 2
    sem_pred[point_cloud.classification==6] = 3
    sem_pred[point_cloud.classification==7] = 2
    point_cloud.classification = sem_pred
    #=========================================================
    # semantic colours
    colours = np.zeros((len(point_cloud),3))

    colours[point_cloud.classification == 0] = np.array([0,0,0])
    colours[point_cloud.classification == 1] = np.array([0,0,1])
    colours[point_cloud.classification == 2] = np.array([0,1,0])
    colours[point_cloud.classification == 3] = np.array([1,0,0])

    pcd_sem.colors = o3d.utility.Vector3dVector(colours)

    #=========================================================
    # instance colours
    pcd_ins = o3d.geometry.PointCloud()
    pcd_ins.points = o3d.utility.Vector3dVector(point_cloud.xyz)

    inst = np.unique(point_cloud.point_source_id)
    colours_group = np.random.rand(len(inst),3)

    colours = np.zeros((len(point_cloud),3))

    for i in inst:
        colours[point_cloud.point_source_id==i,:] = colours_group[i]

    # No segmented points
    colours[point_cloud.classification == 0] = np.array([0,0,0])
    
    pcd_ins.colors = o3d.utility.Vector3dVector(colours)

    o3d.visualization.draw([pcd_ins], title=str(point_cloud_path))
    o3d.visualization.draw([pcd_sem], title=str(point_cloud_path))


for point_cloud_path in folder_path_heuristic.glob('*' + suffix):
    
    plot_segmentation_heuristic(point_cloud_path)
    plot_errors(folder_path_heuristic_errors.joinpath(point_cloud_path.name))
    plot_segmentation(folder_path_dl.joinpath(point_cloud_path.name))
    plot_errors(folder_path_dl_errors.joinpath(point_cloud_path.name))