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
from collections import defaultdict

class Graph:

  def __init__(self):
    self.nodes = set()
    self.edges = defaultdict(list) # {node: nodes connected with}
    self.distances = {} # {pair of nodes: distance}

  def add_node(self, value):
    self.nodes.add(value)

  def add_edge(self, from_node, to_node, distance):
    if not from_node in self.nodes:
      self.nodes.add(from_node)
    if not to_node in self.nodes:
      self.nodes.add(to_node)
    self.edges[from_node].append(to_node)
    self.distances[(from_node, to_node)] = distance
    #for undirected graph
    self.edges[to_node].append(from_node)
    self.distances[(to_node, from_node)] = distance


def dijkstra(graph, initial, final):

  visited = {initial: 0} # {visited node: distance from start}
  path = {}

  nodes = set(graph.nodes)

  while nodes:

    # remove the nearest node from start

    min_node = None
    for node in nodes:
      if node in visited:
        if min_node is None:
          min_node = node
        elif visited[node] < visited[min_node]:
          min_node = node

    if min_node is None:
      break
    if min_node is final:
      current = final
      shortest_path = [current]
      total_distance = 0
      while current is not initial:
        shortest_path.append(path[current])
        total_distance += graph.distances[(path[current], current)]
        current = path[current]
      return shortest_path[::-1], total_distance

    nodes.remove(min_node)

    # update distance from start of nodes connected with the removed node

    current_distance = visited[min_node]

    for edge in graph.edges[min_node]:
      distance = current_distance + graph.distances[(min_node, edge)]
      if edge not in visited or distance < visited[edge]:
        visited[edge] = distance
        path[edge] = min_node

  waypoints = []
  waypoint = final

  if visited[final] > 0:
    while path[waypoint] != initial:
        waypoints.append(waypoint)
        waypoint = path[waypoint]

    waypoints.append(waypoint)
    waypoints.append(initial)

    waypoints.reverse()

    pathlength = visited[final]

    return waypoints, pathlength
  
  return None, 0
