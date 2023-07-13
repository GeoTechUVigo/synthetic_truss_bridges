import pathlib
from Graph import Graph
import laspy
import open3d as o3d
import numpy as np


FOLDER_PC = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/nodes/point_clouds'
FOLDER_GRAPH_GT = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/nodes/graphs_gt'
FOLDER_GRAPH_PRED = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/nodes/graphs_pred'

FOLDER_PC = pathlib.Path(FOLDER_PC)
FOLDER_GRAPH_GT = pathlib.Path(FOLDER_GRAPH_GT)
FOLDER_GRAPH_PRED = pathlib.Path(FOLDER_GRAPH_PRED)

for file in FOLDER_GRAPH_PRED.iterdir():
    graph_pred = Graph.read(file)
    graph_gt = Graph.read(FOLDER_GRAPH_GT.joinpath(file.name))
    las = laspy.read(FOLDER_PC.joinpath(file.name).with_suffix('.laz'))

    # visualise
    _, edges_pred = graph_pred.get_pcd()
    _, edges_gt = graph_gt.get_pcd()
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.cuda.pybind.utility.Vector3dVector(las.xyz)

    # Instance point cloud. Random colours.
    colours = np.zeros((len(las.instances),3))
    inst = np.unique(las.instances)
    colours_group = np.random.rand(len(inst),3)
    for i in inst:
        colours[las.instances==i,:] = colours_group[inst==i][0]

    pcd.colors = o3d.utility.Vector3dVector(colours)

    o3d.visualization.draw([edges_pred, edges_gt, pcd])