import pathlib
from Graph import Graph
import laspy
import open3d as o3d
import numpy as np


FOLDER_PC_TRUE = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/edges/false'
PC_PATH = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/edges/original.laz'
FOLDER_PC_TRUE = pathlib.Path(FOLDER_PC_TRUE)

# point cloud
las = laspy.read(PC_PATH)
pcd_all = o3d.geometry.PointCloud()
pcd_all.points = o3d.cuda.pybind.utility.Vector3dVector(las.xyz)

for file in FOLDER_PC_TRUE.iterdir():
    las_beam = laspy.read(file)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.cuda.pybind.utility.Vector3dVector(las_beam.xyz)
    
    red = np.zeros((len(las_beam.xyz),3))
    red[:,0] = 1.0
    pcd.colors = o3d.cuda.pybind.utility.Vector3dVector(red)

    o3d.visualization.draw([pcd, pcd_all])