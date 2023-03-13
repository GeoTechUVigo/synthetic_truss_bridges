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

from pathlib import Path
import numpy as np
import pandas as pd


class ProfileMesh(object):

    def __init__(self, designation, folderPath="standarized_profiles_formatted"):

        profileType = designation.split('-')[0]

        [h, b, s, t] = ReadProfileParameters(folderPath, designation)
        self.vertices, self.triangles = GenerateMesh(h, b, s, t, profileType)


def ReadProfileParameters(folderPath, designation):
    profileType = designation.split('-')[0]
    filePath = GetProfileCsvPath(folderPath, profileType)
    profileDataFrame = pd.read_csv(filePath)
    profileRow = profileDataFrame.loc[profileDataFrame['Designation'] == designation]
    g = profileRow["G(kg/m)"].iloc[0]
    h = profileRow["h(mm)"].iloc[0] /1000.0 # metres
    b = profileRow["b(mm)"].iloc[0] /1000.0 # metres
    s = profileRow["s(mm)"].iloc[0] /1000.0 # metres
    t = profileRow["t(mm)"].iloc[0] /1000.0 # metres
    a = profileRow["A(cm2)"].iloc[0]
    return [h, b, s, t]


def GenerateOrderedIPoints(h, b, s, t):
    d = (b - s) / 2.0
    points = np.asarray([[0, 0], [d, 0], [b - d, 0], [b, 0],
                        [b, t], [b - d, t], [b - d, h - t], [b, h - t],
                        [b, h], [b - d, h], [d, h], [0, h],
                        [0, h - t], [d, h - t], [d, t], [0, t]])
    return points


def GenerateITriangles():
    triangles = np.asarray([[0, 15, 14], [0, 14, 1], [1, 14, 5], [1, 5, 2],
                            [2, 5, 4], [2, 4, 3], [14, 13, 6], [14, 6, 5],
                            [12, 11, 10], [12, 10, 13], [13, 10, 9],
                            [13, 9, 6], [6, 9, 8], [6, 8, 7]])
    return triangles


def GenerateOrderedUPoints(h, b, s, t):
    points = np.asarray([[0, 0], [s, 0], [b, 0], [b, t],
                        [s, t], [s, h - t], [b, h - t], [b, h],
                        [s, h], [0, h], [0, h - t], [0, t]])
    return points


def GenerateUTriangles():
    triangles = np.asarray([[0, 11, 4], [0, 4, 1], [1, 4, 3], [1, 3, 2], [11, 10, 5],
                            [11, 5, 4], [10, 9, 8], [10, 8, 5], [5, 8, 7], [5, 7, 6]])
    return triangles


def GenerateOrderedTPoints(h, b, s, t):
    d = (b - s) / 2.0
    points = np.asarray([[0, 0], [d, 0], [b - d, 0], [b, 0], [b, t],
                        [b - d, t], [b - d, h], [d, h], [d, t], [0, t]])
    return points


def GenerateTTriangles():
    triangles = np.asarray([[0, 9, 8], [0, 8, 1], [1, 8, 5], [1, 5, 2],
                            [2, 5, 4], [2, 4, 3], [8, 7, 6], [8, 6, 5]])
    return triangles


def GenerateOrderedLPoints(h, b, s, t):
    points = np.asarray([[0, 0], [s, 0], [b, 0], [b, t],
                        [s, t], [s, h], [0, h], [0, t]])
    return points

def GenerateLTriangles():
    triangles = np.asarray([[0,7,4],[0,4,1],[1,4,3],
                            [1,3,2],[7,6,5],[7,5,4]])
    return triangles

def GenerateOrderedRectPoints(h, b, s, t):
    points = np.asarray([[0,0],[s,0],[b-s,0],[b,0],
                         [b,t],[b,h-t],[b,h],[b-s,h],
                         [s,h],[0,h],[0,h-t],[0,t],
                         [s,t],[b-s,t],[b-s,h-t],[s,h-t]])
    return points

def GenerateRectTriangles():
    triangles = np.asarray([[0,12,1],[0,11,12],[1,13,2],[1,12,13],
                            [2,4,3],[2,13,4],[4,14,5],[4,13,14],
                            [5,7,6],[5,14,7],[7,15,8],[7,14,5],
                            [10,8,15],[10,9,15],[11,15,12],[11,10,15]])
    return triangles


def GetProfileCsvPath(folderPath, profileType):
    files = [file.name for file in Path(folderPath).iterdir()]
    csvPath = [file.split('_')[0] for file in files]
    fileIndex = csvPath.index(profileType)
    fileName = files[fileIndex]
    filePath = str(Path(folderPath).joinpath(fileName))

    return filePath


def GenerateMesh(h, b, s, t, profileType):
    if profileType in ["IPN", "HD", "HL", "HLZ", "HP"]:
        # Type I or H
        vertices = GenerateOrderedIPoints(h, b, s, t)
        triangles = GenerateITriangles()
        return vertices, triangles
    elif profileType in ["CH", "UB", "UBP", "UC", "U", "UPE", "UPN","PFC"]:
        # Type C or U
        vertices = GenerateOrderedUPoints(h, b, s, t)
        triangles = GenerateUTriangles()
        return vertices, triangles
    elif profileType in ["L"]:
        vertices = GenerateOrderedLPoints(h,b,s,t)
        triangles = GenerateLTriangles()
        return vertices,triangles
    elif profileType in ["HSS"]:
        vertices = GenerateOrderedRectPoints(h,b,s,t)
        triangles = GenerateRectTriangles()
        return vertices,triangles
    elif profileType in ["T"]:
        vertices = GenerateOrderedTPoints(h,b,s,t)
        triangles = GenerateTTriangles()
        return vertices,triangles
    else:
        print("WHAT?!")

    return None