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
from Util.util import m_to_pygame, clamp
from Objects.human import Human

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

class RadarLine:
    def __init__(self, ori):
        self.ori = ori
        self.step_size = 0.49
        self.max_length = 5.0

    def get_intercepted_edge(self, location, env_map, humans):
        for i in range(0, round(self.max_length/self.step_size)):
            test_spot = [location[0]+(i+1)*self.step_size*np.cos(self.ori), location[1]+(i+1)*self.step_size*np.sin(self.ori)]
            for wall in env_map.walls:
                if intersect(location, test_spot, wall.loc_from, wall.loc_to):
                    if wall.discovered == False:
                        wall.discovered = True
                        env_map.discovery_flag = True
                        #print("DISCOVERY ", wall.loc_from, ", ", wall.loc_to)
                    return
            #Valid spot:
            rounded_spot = [clamp(round(test_spot[0])-1, 0, 18), clamp(round(test_spot[1])-1, 0, 18)]
            if env_map.squares[rounded_spot[1]][rounded_spot[0]].discovered == False:
                for human in humans:
                    if human.location == rounded_spot:
                        #print("FOUND HUMAN")
                        human.discovered = True
                env_map.squares[rounded_spot[1]][rounded_spot[0]].discovered = True
    
    def draw(self, sf, location):
        pygame.draw.line(sf, (128, 0, 0), location, [location[0]+self.max_length*np.cos(self.ori), location[1]+self.max_length*np.sin(self.ori)], 2)

class Robot:
    def __init__(self, location, ori):
        self.location = location
        self.ori = ori
        self.speed = 0.001
        self.visibility = 1.
        self.close_distance = 0.2
        self.waypoints = []
        self.goal_reached = True

        self.radar_lines = []
        n_lines = 8
        for i in range(n_lines):
            self.radar_lines.append(RadarLine(i*2.*np.pi/n_lines))

    def draw(self, sf):
        #pygame.draw.circle(sf, (128, 128, 128, 0.5), m_to_pygame(self.location), self.visibility*25)
        # for radar_line in self.radar_lines:
        #     radar_line.draw(sf, self.location)
        pygame.draw.circle(sf, 128, m_to_pygame(self.location), 5)
    
    def update_position(self, dt):
        if len(self.waypoints) == 0:
            self.goal_reached = True
            dx = 0
            dy = 0
        else:
            self.ori = np.arctan2(self.waypoints[0][1] - self.location[1], self.waypoints[0][0] - self.location[0])
            if np.linalg.norm([self.waypoints[0][0] - self.location[0], self.waypoints[0][1] - self.location[1]]) < self.close_distance:
                self.waypoints.pop(0)
                if len(self.waypoints) > 0:
                    #print(self.waypoints)
                    self.ori = np.arctan2(self.waypoints[0][1] - self.location[1], self.waypoints[0][0] - self.location[0])
            dx = dt*np.cos(self.ori)*self.speed
            dy = dt*np.sin(self.ori)*self.speed
        self.location[0] += dx
        self.location[1] += dy

    def check_for_human(self, humans):
        for human in humans:
            if np.linalg.norm([human.location[0] - self.location [0], human.location[1] - self.location[1]]) < self.visibility:
                human.discovered = True

    def check_for_walls(self, env_map, humans):
        for radar_line in self.radar_lines:
            wall = radar_line.get_intercepted_edge(self.location, env_map, humans)
            # if wall is not None:
            #     if wall.discovered == False:
            #         wall.discovered = True
            #         env_map.discovery_flag = True
            #         print("DISCOVERY ", wall.loc_from, ", ", wall.loc_to)

        # for wall in env_map.walls:
        #     if wall.discovered == False:
        #         #print(np.linalg.norm([wall.loc_from[0] - self.location[0], wall.loc_from[1] - self.location[1]]))
        #         if np.linalg.norm([wall.loc_from[0] - self.location[0], wall.loc_from[1] - self.location[1]]) < self.visibility or np.linalg.norm([wall.loc_to[0] - self.location[0], wall.loc_to[1] - self.location[1]]) < self.visibility:
        #             wall.discovered = True
        #             env_map.discovery_flag = True
        #             print("DISCOVERY ", wall.loc_from, ", ", wall.loc_to)

    def update(self, dt, humans, env_map):
        self.check_for_human(humans)
        self.check_for_walls(env_map, humans)
        self.update_position(dt)
        if self.goal_reached == True:
            self.calculate_new_target(env_map)

    def set_waypoint(self, env_map, goal):
        self.goal = goal
        if self.closest_node(env_map) != goal:
            self.goal_reached = False
            self.waypoints.append(self.closest_node(env_map))
            self.waypoints.extend(env_map.find_path(self.closest_node(env_map), goal)[0])
        else:
            #self.calculate_new_target(env_map)
            self.goal_reached = True

    def closest_node(self, env_map):
        return [round(self.location[0]), round(self.location[1])]

    def reroute(self, env_map):
        self.waypoints = []
        self.set_waypoint(env_map, self.goal)

    def calculate_new_target(self, env_map):
        #At target location: Calculate clusters and find closest
        #print("Calculating new clusters")
        env_map.calculate_clusters()
        new_loc = env_map.get_unallocated_cluster_location()
        if new_loc is not None:
            self.goal = new_loc
            self.goal_reached = False
            #print("New goal:")
            #print(self.goal)
            self.waypoints.append(self.closest_node(env_map))
            self.waypoints.extend(env_map.find_path(self.closest_node(env_map), self.goal)[0])

    def assign_cluster(self, cluster_index, env_map):
        self.waypoints = []
        new_loc = env_map.clusters[cluster_index].mean
        if new_loc is not None:
            self.goal = new_loc
            self.goal_reached = False
            #print("New goal:")
            #print(self.goal)
            self.waypoints.append(self.closest_node(env_map))
            self.waypoints.extend(env_map.find_path(self.closest_node(env_map), self.goal)[0])
    
    def calculate_path_length(self, target, env_map):
        return env_map.find_path(self.closest_node(env_map), target)[1]
