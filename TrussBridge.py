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

import stat
import open3d as o3d
import numpy as np
import laspy

from Beam import Beam
from Graph import Graph


class TrussBridge(object):


    def __init__(self, node_coordinates, nodes, profile, beam_orient, classification, deck, centre, orientation, density, cameras) -> object:
        """
        Class for the generation of truss bridges. Each beam of the bridge is a Beam object.

        :param node_coordinates: (nº nodes,3) node coordinates of the Truss centred in [0,0,0] and oriented in X axis.
        :param nodes: (nº beams, 2) with the start and end index node of each beam.
        :param profile: (nº beams,) with the information of the profile of each beam.
        :param beam_orient: (nº beams,) with the information of the orientation.
        :param classification: (nº beams,) of uint with the classification of each beam. Class 0 is reserved to deck.
        :param deck: [start_point, end_point, [width, height]] of the deck. Start and end points are in the middle of the profile of the deck. 
                      Hieght is measured from the bottom of the truss to the bottom of the deck.
        :param centre: (3,) centre of the Truss.
        :param orientation: (3,) orientation of the Truss expresed as ZYX rotation angles.
        :param density: density of the point cloud (points/m²).
        :param cameras: list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px'].
        """
    
        self.name = type(self).__name__
        self.nodes = nodes
        self.profile = profile
        self.classification = classification
        self.beam_orient = beam_orient
        self.density = density
        self.node_coordinates = node_coordinates
        
        # Beam objects and deck are generated in centre=[0,0,0] and orientation=[0,0,0].
        # Then, they are place. It is done to avoid rotation problems.
        self.graph = Graph(node_coordinates[nodes])
        self.beam_setter()
        self.deck = Beam(deck[0], deck[1], ["deck", deck[2][0], deck[2][1]], 0)

        self.centre = centre
        self.orientation = orientation
        self.point_cloud_from_positions(cameras)
        self.place(node_coordinates, deck)

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

    @property
    def nodes(self):
        return self.__nodes

    @nodes.setter
    def nodes(self, nodes):
        self.__nodes = nodes

    @property
    def profile(self):
        return self.__profile

    @profile.setter
    def profile(self, profile):
        self.__profile = profile

    @property
    def classification(self):
        return self.__classification

    @classification.setter
    def classification(self, classification):
        self.__classification = classification        

    @property
    def beam_orient(self):
        return self.__beam_orient

    @beam_orient.setter
    def beam_orient(self, beam_orient):
        self.__beam_orient = beam_orient 

    @property
    def density(self):
        return self.__density

    @density.setter
    def density(self, density):
        self.__density = density 

    @property
    def node_coordinates(self):
        return self.__node_coordinates

    @node_coordinates.setter
    def node_coordinates(self, node_coordinates):
        self.__node_coordinates = node_coordinates 

    @property
    def beam(self):
        return self.__beam

    @beam.setter
    def beam(self, beam):
        self.__beam = beam 

    @property
    def graph(self):
        return self.__graph

    @graph.setter
    def graph(self, graph):
        self.__graph = graph 

    ###############################################################################
    # Methods


    def beam_setter(self):
        '''
        Method for setting beam objects.
        '''
        beam = np.ones(len(self.nodes), dtype='object')
        for i in range(len(beam)):
            beam[i] = Beam(self.node_coordinates[self.nodes[i,0]], self.node_coordinates[self.nodes[i,1]], 
                           self.profile[i], self.beam_orient[i])
        
        self.beam = beam


    def place(self, node_coordinates, deck):
        '''
        Method to place the nodes, beam.mesh, beam.point_cloud, deck.mesh and deck.point_cloud 
        in the centre and orientation specified.
        '''

        # Create PointCloud object with node_coordinates and deck points
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.concatenate((node_coordinates, np.concatenate((deck[0],deck[1])).reshape(-1,3))))

        # Points to the new centre and orientation
        T = np.eye(4)
        T[:3, :3]  = o3d.geometry.get_rotation_matrix_from_zyx((self.orientation[0], self.orientation[1], self.orientation[2]))
        T[:3,-1] = self.centre

        # Transform
        pcd = pcd.transform(T)

        # Write properties
        points = np.asarray(pcd.points)
        self.node_coordinates = points[:-2]

        for i in range(len(self.beam)):
            self.beam[i].mesh = self.beam[i].mesh.transform(T)
            self.beam[i].point_cloud = self.beam[i].point_cloud.transform(T)

        self.deck.mesh = self.deck.mesh.transform(T)
        self.deck.point_cloud = self.deck.point_cloud.transform(T)


    def mesh(self) -> o3d.geometry.TriangleMesh():
        """
        Returns all the meshes of the self.beam objects plus self.deck.
        """
        mesh = np.array([])
        for this_beam in self.beam:

            mesh = np.append(mesh, this_beam.mesh)
        
        mesh = np.append(mesh, self.deck.mesh)

        return np.sum(mesh)


    def point_cloud(self) -> o3d.geometry.PointCloud():
        """
        Returns all the point clouds of the self.beam objects plus self.deck.
        """
        point_cloud = np.array([])
        for this_beam in self.beam:
            point_cloud = np.append(point_cloud, this_beam.point_cloud)
        
        point_cloud = np.append(point_cloud, self.deck.point_cloud)

        return np.sum(point_cloud)


    def get_class_and_ins(self) -> np.ndarray:
        """"
        Function to calculate the classification and instnace arrays.
        The classification numbers are:
            deck = 0
            beam[i] = self.classification[i]

        :return:
            classification: np.ndarray(nº points) with the information of the class of each point.
            instance: np.ndarray(nº points) with the information of the instance number of each point.
        """

        # Calcualte the number of points.
        n_points = 0
        for this_beam in self.beam:
            n_points += len(np.asarray(this_beam.point_cloud.points))

        n_points += len(np.asarray(self.deck.point_cloud.points))

        # Initialise arrays
        classification = np.zeros(n_points)
        instance = np.zeros(n_points)

        # Initialise
        idx_new = 0
        idx = 0 
        group = 0

        # Instance and classification of each point
        for i in range(len(self.beam)):  
            # Update idx_new
            idx_new += len(np.asarray(self.beam[i].point_cloud.points))

            # Classification number
            n_class = self.classification[i]

            # Class
            classification[idx:idx_new] = n_class

            # Group id
            instance[idx:idx_new] = group

            # Update idx
            idx = idx_new

            # Update group id
            group += 1

        # deck
        idx_new += len(np.asarray(self.deck.point_cloud.points))
        classification[idx:idx_new] = 0
        instance[idx:idx_new] = group

        return classification, instance


    def label_nodes(self, distance) -> np.ndarray:
        '''
        Method to label the points by their distance to the nodes.

        :param distance: maximum distance between a point and a node to be labelled as node point.
        '''
        

        xyz = np.asarray(self.point_cloud().points)
                    
        classification = np.zeros((len(xyz)), dtype='int')
        instance = np.zeros((len(xyz)), dtype='int')

        idx = 0
        for node in self.node_coordinates:

                idx += 1

                idx_node = np.linalg.norm(xyz - node, axis=1) < distance

                classification[idx_node] = 1
                instance[idx_node] = idx

        return classification, instance


    def save_las(self, path:str, version:float = 1.4, point_format:int = 6, scale:float = 0.01, distance_nodes:float = None):
        """"
        Function to save to the point cloud as LAS or LAZ.
        User_data field containd the unique id of each beam
        Classification filed contains the type of each beam, being deck = 0 and the beams with their specified index.

        :param path: string of the path to save the LAS.
        :param version: version of the LAS. 1.4 by default.
        :param point_format: point format of the LAS. 6 by default.
        :param scale: scale of LAS. 0.01 by default.
        :param distance_nodes: if specified, those points closer to a node than this parameters are labeled with 1.
        """

        if not distance_nodes is None:
            classification_node, _ = self.label_nodes(distance_nodes)


        classification, instance = self.get_class_and_ins()

        # Locations
        location = np.asarray(self.point_cloud().points)

        # LAS object
        las = laspy.create(point_format=point_format,file_version=str(version))
        # Offset and scale
        las.header.offset = np.mean(np.concatenate((location.max(axis=0).reshape(-1,1),location.min(axis=0).reshape(-1,1)),axis=1),axis=1).tolist()
        las.header.scale = [scale,scale,scale]

        # Write in las object.
        las.X = (location[:,0] - las.header.offsets[0]) / las.header.scales[0]
        las.Y = (location[:,1] - las.header.offsets[1]) / las.header.scales[1]
        las.Z = (location[:,2] - las.header.offsets[2]) / las.header.scales[2]
        las.classification = classification
        # las.user_data = instance
        las.add_extra_dim(laspy.point.format.ExtraBytesParams('instances', 'uint16'))
        setattr(las, 'instances', instance) 
        

        if not distance_nodes is None:
            las.add_extra_dim(laspy.point.format.ExtraBytesParams('nodes', 'uint8')) # non sei como gardalo como boolean
            setattr(las, 'nodes', classification_node)         
        
        # Save
        las.write(path)


    def save_nodes_las(self, path:str, version:float = 1.4, point_format:int = 6, scale:float = 0.01):
        """"
        Function to save the node coordinates as LAS file.

        :param path: string of the path to save the LAS.
        :param version: version of the LAS. 1.4 by default.
        :param point_format: point format of the LAS. 6 by default.
        :param scale: scale of LAS. 0.01 by default.
        """

        # Locations
        location = np.asarray(self.node_coordinates)

        # LAS object
        las = laspy.create(point_format=point_format,file_version=str(version))
        # Offset and scale
        las.header.offset = np.mean(np.concatenate((location.max(axis=0).reshape(-1,1),location.min(axis=0).reshape(-1,1)),axis=1),axis=1).tolist()
        las.header.scale = [scale,scale,scale]

        # Write in las object.
        las.X = (location[:,0] - las.header.offsets[0]) / las.header.scales[0]
        las.Y = (location[:,1] - las.header.offsets[1]) / las.header.scales[1]
        las.Z = (location[:,2] - las.header.offsets[2]) / las.header.scales[2]
        
        # Save
        las.write(path)


    @staticmethod
    def update_nodes(beam_nodes, beam_sect, beam_orient, beam_sem, node_coordinates, beam_p0, beam_p1, beam_data):
        '''
        Method used by child class to concatenate the data of new beams.

        :param beam_nodes: current beam nodes array.
        :param beam_sect: current beam profiles array.
        :param beam_orient: current beam orientation array.
        :param beam_sem: current beam classification array.
        :param node_coordinates: current node coordinates array.
        :param beam_p0: new start nodes.
        :param beam_p1: new end nodes.
        :param beam_data: new beam information.
        :return:
            node_coordinates: node_coordinates updated.
            beam_nodes: beam_nodes updated.
            beam_sect: beam_sect updated.
            beam_orient: beam_orient updated.
            beam_sem: beam_sem updated.
        '''

        n_beams = len(beam_p0)

        beam_nodes = np.concatenate((beam_nodes,np.arange(n_beams).reshape(-1,1) + [len(node_coordinates), len(node_coordinates)+ n_beams]))
        beam_sect = np.concatenate((beam_sect, np.asarray(beam_data[0])*np.ones(n_beams,dtype='object')))
        beam_orient = np.concatenate((beam_orient, np.asarray(beam_data[1])*np.ones(n_beams)))
        beam_sem = np.concatenate((beam_sem, np.asarray(beam_data[2])*np.ones(n_beams, dtype='uint')))

        node_coordinates = np.concatenate((node_coordinates, np.concatenate((beam_p0, beam_p1))))

        return node_coordinates, beam_nodes, beam_sect, beam_orient, beam_sem

    
    def show_pc(self):
        '''
        Plot two point cloud, one with colours that represent the semantic classes
        and the other that represents the instances classes.
        The wireframe from the mesh is alos plotted.
        Semantic colours:
            0 -> [0.5, 0.5, 0.5]
            1 -> [0, 0, 1]
            2 -> [0, 1, 0]
            3 -> [1, 0, 0]
        
        Instance colour: random
        '''

        # Indexes
        classification, instances = self.get_class_and_ins()

        # Semantic point cloud.
        pcd_sem = self.point_cloud()
        colours = np.zeros((len(classification),3))
        colours[classification == 0] = np.array([0.5,0.5,0.5])
        colours[classification == 1] = np.array([0,0,1])
        colours[classification == 2] = np.array([0,1,0])
        colours[classification == 3] = np.array([1,0,0])
        pcd_sem.colors = o3d.utility.Vector3dVector(colours)

        # Instance point cloud. Random colours.
        pcd_ins = self.point_cloud()
        colours = np.zeros((len(classification),3))
        instances = instances.astype('int')
        inst = np.unique(instances)
        colours_group = np.random.rand(len(inst),3)
        for i in inst:
            colours[instances==i,:] = colours_group[inst==i][0]

        pcd_ins.colors = o3d.utility.Vector3dVector(colours)

        #Wireframe
        line= o3d.geometry.LineSet.create_from_triangle_mesh(self.mesh())

        o3d.visualization.draw([pcd_ins, pcd_sem, line])


    def point_cloud_from_positions(self, cameras):
        '''
        Method for generating a point cloud from the mesh by specifing the parameters of the LiDAR as it is a camera.
        If camera is None, the point cloud is generated with random points.

        :param cameras: list of dicts with the following keys for each LiDAR position: ['fov_deg', 'center', 'eye', 'up', 'width_px', 'height_px'].
        '''

        if cameras is None:
            for i in range(len(self.beam)):
                self.beam[i].point_cloud  = self.beam[i].calculate_point_cloud(self.density)
            self.deck.point_cloud = self.deck.calculate_point_cloud(self.density)
            return
        
        # create scene        
        scene = o3d.t.geometry.RaycastingScene()

        # Add each member
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(self.deck.mesh)) # member 0 is the deck
        for element in self.beam:
            scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(element.mesh))

        points_total = np.zeros((0,3))
        idx_member_total = np.zeros((0), dtype='int')

        for camera in cameras:
            # create rays
            rays = scene.create_rays_pinhole(fov_deg=camera['fov_deg'],
                                    center=camera['center'],
                                    eye=camera['eye'],
                                    up=camera['up'],
                                    width_px=camera['width_px'],
                                    height_px=camera['height_px'])
            ans = scene.cast_rays(rays)

            # collision
            hit = ans['t_hit'].isfinite()

            # points
            points = (rays[hit][:,:3] + rays[hit][:,3:]*ans['t_hit'][hit].reshape((-1,1))).numpy()

            # idx of the member of each point
            idx_member = ans['geometry_ids'][hit].numpy()

            # Add
            points_total = np.concatenate((points_total, points))
            idx_member_total = np.concatenate((idx_member_total, idx_member))

        unique_idxs = np.unique(idx_member_total)

        for idx in range(len(self.beam)+1):
            
            pcd = o3d.geometry.PointCloud()
            if np.any(np.in1d(unique_idxs, idx)):
                pcd.points = o3d.cuda.pybind.utility.Vector3dVector(points_total[idx_member_total == idx])

            # idx 0 is the deck.
            if idx == 0:
                self.deck.point_cloud = pcd
            else:
                self.beam[idx-1].point_cloud = pcd
