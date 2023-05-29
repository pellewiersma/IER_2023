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
from Objects.map import Map
from Objects.robot import Robot
from Objects.human import Human
import time
from scipy.optimize import linear_sum_assignment

pygame.init()
clock = pygame.time.Clock()
# Set up the drawing window
screen = pygame.display.set_mode([500, 500])
for iteration in range(30):

    env_map = Map('Scenarios/map_empty.bmp')

    audio_heuristic = True

    human = Human([3.5, 3.5])
    humans = []
    for i in range(4):
        humans.append(Human([np.random.randint(1, 18), np.random.randint(1, 18)]))


    robots = []
    for i in range(4):
        robots.append(Robot([19.0, 2.], np.pi/2.0))

    if audio_heuristic == True:
        cost = np.zeros((len(robots),4))
        if len(humans) == 4:
            for i in range(4):
                for j in range(4):
                    cost[i,j] = robots[i].calculate_path_length([round(humans[j].location[0]), round(humans[j].location[1])], env_map)
            row_ind, col_ind = linear_sum_assignment(cost)
            for i in range(len(robots)):
                robots[i].set_waypoint(env_map, [round(humans[col_ind[i]].location[0]), round(humans[col_ind[i]].location[1])])
    else:
        robots[0].set_waypoint(env_map, [3, 3])
        robots[1].set_waypoint(env_map, [15, 3])
        robots[2].set_waypoint(env_map, [3, 15])
        robots[3].set_waypoint(env_map, [15, 15])

    # Run until the user asks to quit
    start_time = time.perf_counter()
    running = True
    while running:

        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
        #Draw the environment

        # Fill the background with white
        screen.fill((255, 255, 255))
        
        #Draw the map
        env_map.draw(screen)
        for robot in robots:
            robot.draw(screen)

        humans_found = True
        for human in humans:
            human.draw(screen)
            if human.discovered == False:
                humans_found = False

        if humans_found == False:
            dt = clock.tick(25)
            for robot in robots:
                robot.update(dt, humans, env_map)
            if env_map.discovery_flag == True:
                env_map.update_graph()
                if env_map.n_clusters < 4 or audio_heuristic == True:
                    for robot in robots:
                        robot.reroute(env_map)
                else:
                    cluster_list = env_map.clusters[:4]
                    cost = np.ones((len(robots), 4))
                    for i in range(len(robots)):
                        for j in range(4):
                            cost[i, j] = robots[i].calculate_path_length(cluster_list[j].mean, env_map)
                    row_ind, col_ind = linear_sum_assignment(cost)
                    for i in range(len(robots)):
                        robots[i].assign_cluster(col_ind[i], env_map)
                env_map.discovery_flag = False
        else:
            end_time = time.perf_counter()
            print("{:.1f}, ".format(end_time-start_time))
            running = False

        # Flip the display due to difference in origin definition
        pygame.display.flip()

# Done! Time to quit.
pygame.quit()
