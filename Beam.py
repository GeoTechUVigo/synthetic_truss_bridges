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

import open3d as o3d
import numpy as np
from utils import angle_between_2Dvectors
from utils import angle_between_vectors
from ProfileMesh import ProfileMesh


class Beam(object):


    def __init__(self, point_0, point_1, profile, orientation) -> object:
        """
        Constructor of Beam class. Calculates the direction, length, centre, orientation and profile of a beam.
        Beam objects can have its mesh and point cloud using its specific functions.

        :param point_0: start point of the beam.
        :param point_1: end point of the beam.
        :param profile: profile type of the beam.
        :param orientation: orientation of the beam.
        """

        #TODO: soamente se actualiza mesh despois do metodo place() en TrussBridge. O resto non e fiable. Unicamente o point_o e point_1 de DECK por outros motivos. Ver como resolver isto.
                
        self.point_0 = np.asarray(point_0)
        self.point_1 = np.asarray(point_1)

        self.length = np.linalg.norm(point_1 - point_0)
        self.direction = (point_1 - point_0) / self.length
        self.centre = np.mean(np.concatenate((point_1.reshape(1,-1), point_0.reshape(1,-1)), axis=0), axis=0)
        self.orientation = orientation
        self.profile = profile

        meshResult = self.__calculate_mesh()
        self.mesh = meshResult[0]
        self.rotM= meshResult[1]

        # The point cloud is calculated using TrussBridge.point_cloud_from_positions()
        self.point_cloud = None

    # Properties
    @property
    def length(self):
        return self.__length

    @length.setter
    def length(self, length):
        self.__length = length

    @property
    def direction(self):
        return self.__direction

    @direction.setter
    def direction(self, direction):
        self.__direction = direction

    @property
    def centre(self):
        return self.__centre

    @centre.setter
    def centre(self, centre):
        self.__centre = centre

    @property
    def orientation(self):
        return self.__orientation

    @orientation.setter
    def orientation(self, orientation):
        self.__orientation = orientation

    @property
    def profile(self):
        return self.__profile

    @profile.setter
    def profile(self, profile):
        self.__profile = profile

    @property
    def mesh(self):
        return self.__mesh

    @mesh.setter
    def mesh(self, mesh):
        self.__mesh = mesh

    @property
    def point_cloud(self):
        return self.__point_cloud

    @point_cloud.setter
    def point_cloud(self, point_cloud):
        self.__point_cloud = point_cloud

    @property
    def area(self):
        return self.__area

    @area.setter
    def area(self, area):
        self.__area = area

    #################################################################################
    # Methods

    def __calculate_area(self) -> float:
        """
        Function to calculate the area of the mesh. The area of the mesh is computed as the sum of the areas of 
        the triangles of the mesh. The areas are computed using Heron's formula.
        """

        # Vertices coordinates and indexes of each vertices for each triangle.
        triangles = np.asarray(self.mesh.triangles)
        vertices = np.asarray(self.mesh.vertices)

        # Coordinates of each triangle
        points = vertices[triangles]

        # Lenghts of each side
        a = np.linalg.norm(points[:,0,:] - points[:,1,:], axis=1)
        b = np.linalg.norm(points[:,1,:] - points[:,2,:], axis=1)
        c = np.linalg.norm(points[:,2,:] - points[:,0,:], axis=1)

        # Half perimiter
        s = np.sum((a,b,c), axis=0)/2

        # Heron's formula
        area = (s*(s-a)*(s-b)*(s-c))**0.5

        # Total area
        area = np.sum(area)
        
        return area


    def __complete_mesh_triangles(triangles_bottom) -> np.ndarray:
        """
        This functions calculates the indexes of the triangles in the top face and in the laterals faces of an extruded mesh
        given the indexes of the vertices of the triangles in the bottom face.

        :param triangles_bottom: (N,3) np.ndarray with the indexes of the vertices of the triangles in the bottom face.
        """

        number_points = triangles_bottom.max() + 1
        
        # Triangles in the top face
        triangles_top = triangles_bottom + number_points
        triangles_top[:,[0,1]] = triangles_top[:,[1,0]]

        # Triangles in the lateral faces
        triangles_lat = np.concatenate((np.concatenate((np.arange(number_points-1).reshape(-1,1), 
                                                       (np.arange(number_points-1) + 1).reshape(-1,1),
                                                       (np.arange(number_points-1) + number_points).reshape(-1,1)), axis=1),
                                        np.concatenate(((np.arange(number_points-1) + number_points + 1).reshape(-1,1), 
                                                        (np.arange(number_points-1) + number_points).reshape(-1,1),
                                                        (np.arange(number_points-1) + 1).reshape(-1,1)), axis=1),
                                        np.array([[0, 2*number_points - 1, number_points - 1], 
                                                  [0, number_points, 2*number_points - 1]])),axis=0)

        triangles = np.concatenate((triangles_bottom, triangles_top, triangles_lat),axis=0)

        return triangles


    def __calculate_mesh(self) -> o3d.geometry.TriangleMesh():
        """
        Generates an oriented mesh.
        It extrudes the mesh's profile and orients it using the direction of the beam.
        """

        if isinstance(self.profile, list):
            if self.profile[0] == 'deck':
                vertices = np.array([[0,0], [self.profile[2],0], [self.profile[2], self.profile[1]], [0,self.profile[1]]])
                triangles = np.array([[ 2, 1, 0], [ 2, 0, 3]]) # Bottom face triangles

        else:
            profile_mesh = ProfileMesh(designation=self.profile)
            vertices = profile_mesh.vertices
            triangles = profile_mesh.triangles

        vertices = np.concatenate((vertices,np.zeros((len(vertices),1))),axis=1)
        vertices = np.concatenate((vertices, vertices + [0,0, self.length]), axis=0)

        triangles = Beam.__complete_mesh_triangles(triangles)

        # Mesh object
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        mesh.triangles = o3d.utility.Vector3iVector(triangles)

        # Centre
        #mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) - mesh.get_center())
        mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) - (mesh.get_max_bound()-mesh.get_min_bound())/2)
                                                   
        #===========================================================================================
        ## Place the mesh

        rotM = self.__get_translation_matrix(self.direction, self.orientation)
        mesh = mesh.transform(rotM)

        return mesh,rotM

        # coordinates = o3d.geometry.TriangleMesh.create_coordinate_frame()
        # o3d.visualization.draw([mesh, coordinates])

        #TODO: Se le pueden añadir texturas http://www.open3d.org/docs/release/tutorial/geometry/mesh.html#Mesh-properties

    def __get_translation_matrix(self, target_vector, rot_angle):
        """
        Calculate the translation matrix to place the beam.
        The rotation angles are ZYZ.
        """

        # Rotation in Z in rad
        projectionXY = target_vector[[0,1]]
        
        # If the projection is 0, non rotation or pi, depending on self.direction points Z or -Z.
        # It is an agreement to create mirror profiles.
        # The rotation angle is measured from X axis [1,0].
        if np.all(projectionXY == [0,0]):
            if target_vector[2] > 0:
                ccw_angle_from_x_axis = 0.0
            else:
                ccw_angle_from_x_axis = np.pi
        else:
            ccw_angle_from_x_axis = angle_between_2Dvectors([1,0],projectionXY)

        # Rotation in Z in rad
        angle_from_z_to_target = angle_between_vectors([0,0,1],target_vector)

        # Matrix initialization
        T = np.eye(4)

        # Rotate Z then Y then Z again
        R1 = o3d.geometry.get_rotation_matrix_from_zyx((ccw_angle_from_x_axis, angle_from_z_to_target, 0.0))
        R2 = o3d.geometry.get_rotation_matrix_from_zyx((rot_angle, 0.0, 0.0))
        T[:3,:3]= np.matmul(R1,R2)

        T[:3,-1] = self.centre

        return T


    def calculate_point_cloud(self, density) -> o3d.geometry.PointCloud():
        """
        Generates a point cloud.

        :param density: density of the point cloud (points/m²).
        """
        if isinstance(density, type(None)):
            return None
        else:
            return self.mesh.sample_points_uniformly(int(density*self.__calculate_area()))