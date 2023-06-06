import numpy as np
import laspy
import pathlib

version = 1.4
point_format:int = 6
scale:float = 0.01

PATH_IN = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/nodes_las/'
PATH_OUT = '/home/dlamasnovoa/Documents/repositories/synthetic_truss_bridges/data/nodes_labels/'

SUFFIX_IN = '.laz'
SUFFIX_OUT = '.laz'

PATH_IN = pathlib.Path(PATH_IN)
PATH_OUT = pathlib.Path(PATH_OUT)

if not PATH_IN.is_dir(): raise TypeError("{0} does not exist".format(PATH_IN))
if not PATH_OUT.is_dir(): raise TypeError("{0} does not exist".format(PATH_OUT))

for file in PATH_IN.iterdir():
    if not file.suffix == SUFFIX_IN: continue

    # Locations
    xyz = laspy.read(file).xyz

    #TODO: Add noise TO xyz. Gardar as nubes con clasificacion binaria ou con segmentadas como puntos da celosia e puntos ruido?


    # LAS object
    las = laspy.create(point_format=point_format,file_version=str(version))
    # Offset and scale
    las.header.offset = np.mean(np.concatenate((xyz.max(axis=0).reshape(-1,1),xyz.min(axis=0).reshape(-1,1)),axis=1),axis=1).tolist()
    las.header.scale = [scale,scale,scale]

    # Write in las object.
    las.X = (xyz[:,0] - las.header.offsets[0]) / las.header.scales[0]
    las.Y = (xyz[:,1] - las.header.offsets[1]) / las.header.scales[1]
    las.Z = (xyz[:,2] - las.header.offsets[2]) / las.header.scales[2]
        
    # Save
    las.write(pathlib.Path.joinpath(PATH_OUT,file.stem).with_suffix(SUFFIX_OUT))