# Copyright (c) 2023-2024 Pelle Wiersma.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:

# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import pygame
import numpy as np
from Objects.wall import Wall
from Util.dijkstra import dijkstra, Graph
from Util.util import waypoint_to_node, node_to_waypoint, m_to_pygame
from PIL import Image
from Objects.square import Square
from Objects.cluster import Cluster

class Map:
    def __init__(self, map_file):
        self._walls = []
        self._squares = []
        for y in range(19):
            row = []
            for x in range(19):
                row.append(Square(x, y))
            self._squares.append(row)
        self.graph = Graph()
        self.edges = []
        self.map_file = map_file
        self.create_graph()
        self.update_graph()
        self._discovery_flag = False
        self._clusters = []

    def draw(self, sf):
        #for i in range(20):
        #    for j in range(20):
        #        pygame.draw.circle(sf, 128, (i*25, j*25), 1)
        for row in self._squares:
            for cell in row:
                cell.draw(sf)
        for wall in self._walls:
            wall.draw(sf)

        #for edge in self.edges:
        #    pygame.draw.line(sf, (0, 255, 0), m_to_pygame(node_to_waypoint(edge[0])), m_to_pygame(node_to_waypoint(edge[1])))

    def create_graph(self):
        self._walls = []
        self.edges = []

        loaded_map = np.array(Image.open(self.map_file).convert('L')).reshape(41, 41)
        #print(loaded_map.shape)
        loaded_map = np.transpose(loaded_map)
        #print(loaded_map.shape)
        loaded_map = np.fliplr(loaded_map)

        for y in range(41):
            if y % 2 == 1:
                for x in range(0, 41):
                    if x % 2 == 0:
                        if loaded_map[x, y] == 0:
                            wall = Wall([x/2+0.5, y/2], [x/2+0.5, y/2+1])
                            self._walls.append(wall)
            else:
                for x in range(0, 41):
                    #print(x % 2)
                    if x % 2 == 1:
                        #print(x)
                        if loaded_map[x, y] == 0:
                            #print(x, ", ", y)
                            wall = Wall([x/2, y/2+0.5], [x/2+1., y/2+0.5])
                            self._walls.append(wall)


        for y in range(20):
            for x in range(20):
                self.edges.append((waypoint_to_node([x, y]), waypoint_to_node([x+1, y]), 1))
                self.edges.append((waypoint_to_node([x, y]), waypoint_to_node([x, y+1]), 1))

    def update_graph(self):
        self.graph = Graph()
        for wall in self._walls:
            if wall.discovered == True:
                edge_to_remove = self.get_obstructed_edge(wall)
                for edge in self.edges:
                    if edge_to_remove[0] == edge[0] and edge_to_remove[1] == edge[1]:
                        self.edges.remove(edge)
                        #print("removed edge!!!")
                
        for edge in self.edges:
            self.graph.add_edge(*edge)

    def find_path(self, start, finish):
        path = dijkstra(self.graph, waypoint_to_node(start), waypoint_to_node(finish))
        waypoints = []
        #print(start)
        #print(finish)
        if path[1] > 0:
            for point in path[0]:
                waypoints.append(node_to_waypoint(point))
        return waypoints, path[1]

    def get_obstructed_edge(self, wall):
        if wall._from[0] == wall._to[0]:
            #Vertical wall!
            waypoint1 = [round(wall.loc_from[0]-0.5), round(wall.loc_from[1]+0.5)]
            waypoint2 = [round(wall.loc_from[0]+0.5), round(wall.loc_from[1]+0.5)]
        else:
            #Horizontal wall!
            waypoint1 = [round(wall.loc_from[0]+0.5), round(wall.loc_from[1]-0.5)]
            waypoint2 = [round(wall.loc_from[0]+0.5), round(wall.loc_from[1]+0.5)]
        
        #print(waypoint1)
        #print(waypoint2)
        node1 = waypoint_to_node(waypoint1)
        node2 = waypoint_to_node(waypoint2)
        edge = (node1, node2, 1)

        return edge
    
    def calculate_clusters(self):
        self._clusters = []
        for y in range(19):
            for x in range(19):
                self._squares[y][x].assigned_to_cluster = False
        for y in range(19):
            for x in range(19):
                square = self._squares[y][x]
                if square.assigned_to_cluster == False and square.discovered == False: 
                    #Find cluster_loop
                    adjecent_squares_list = [square]
                    square.assigned_to_cluster = True
                    squares_to_check = [square]
                    while len(squares_to_check) > 0:
                        if squares_to_check[0].location[0] > 0:
                            potential_neighbour = self._squares[squares_to_check[0].location[1]][squares_to_check[0].location[0]-1]
                            if potential_neighbour.assigned_to_cluster == False and potential_neighbour.discovered == False:
                                adjecent_squares_list.append(potential_neighbour)
                                potential_neighbour.assigned_to_cluster = True
                                squares_to_check.append(potential_neighbour)
                                #print("added neighbour")
                        if squares_to_check[0].location[0] < 18:
                            potential_neighbour = self._squares[squares_to_check[0].location[1]][squares_to_check[0].location[0]+1]
                            if potential_neighbour.assigned_to_cluster == False and potential_neighbour.discovered == False:
                                adjecent_squares_list.append(potential_neighbour)
                                potential_neighbour.assigned_to_cluster = True
                                squares_to_check.append(potential_neighbour)
                                #print("added neighbour")
                        if squares_to_check[0].location[1] > 0:
                            potential_neighbour = self._squares[squares_to_check[0].location[1]-1][squares_to_check[0].location[0]]
                            if potential_neighbour.assigned_to_cluster == False and potential_neighbour.discovered == False:
                                adjecent_squares_list.append(potential_neighbour)
                                potential_neighbour.assigned_to_cluster = True
                                squares_to_check.append(potential_neighbour)
                                #print("added neighbour")
                        if squares_to_check[0].location[1] < 18:
                            potential_neighbour = self._squares[squares_to_check[0].location[1]+1][squares_to_check[0].location[0]]
                            if potential_neighbour.assigned_to_cluster == False and potential_neighbour.discovered == False:
                                adjecent_squares_list.append(potential_neighbour)
                                potential_neighbour.assigned_to_cluster = True
                                squares_to_check.append(potential_neighbour)
                                #print("added neighbour")
                        squares_to_check.pop(0)
                        #print("done checking this square")
                    #print("finished cluster")
                    #print("Size: {}".format(len(adjecent_squares_list)))
                    cluster = Cluster(adjecent_squares_list)
                    self._clusters.append(cluster)
                    #print(cluster.mean)


        #Sort clusters on size
        self._clusters = sorted(self._clusters, key=lambda cluster: cluster.size(), reverse=True)

        #for cluster in self._clusters:
            #print(cluster.size())

    def get_unallocated_cluster_location(self):
        for cluster in self._clusters:
            if cluster.assigned == False:
                #print("Cluster unassigned")
                cluster.assigned = True
                return cluster.mean
        return None

    @property
    def walls(self):
        return self._walls
    
    @property
    def squares(self):
        return self._squares

    @property
    def discovery_flag(self):
        return self._discovery_flag

    @discovery_flag.setter
    def discovery_flag(self, value):
        self._discovery_flag = value

    @property
    def n_clusters(self):
        return len(self._clusters)
    
    @property
    def clusters(self):
        return self._clusters