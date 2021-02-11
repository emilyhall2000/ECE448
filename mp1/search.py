# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Kelvin Ma (kelvinm2@illinois.edu) on 01/24/2021

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,fast)
from queue import Queue
from typing import Any, List, Dict, Tuple
import heapq
import copy


def bfs(maze):
    """
    Runs BFS for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """

    path: List = []  # the path that will be returned
    frontier: Queue[Any] = Queue(0)  # empty queue that will hold the current nodes needing to be expanded
    explored = {}  # a dictionary of the explored objects
    parents = {}

    # check if start point is a waypoint 
    if maze.start in maze.waypoints:
        path.append(maze.start)
        return path

        # start point is not a waypoint, so BFS
    # not setting parent for the start point bc we will compare parents to the start point
    frontier.put(maze.start)
    explored[maze.start] = True
    # explored.get(key, False) 
    # will return false if the key does not exist, the value otherwise

    while not frontier.empty():
        current: List = frontier.get()
        neighbors = maze.neighbors(current[0], current[1])
        for i in neighbors:
            if not explored.get(i, False):
                parents[i] = current
                if i in maze.waypoints:
                    # constructing the path
                    path.append(i)  # put waypoint at the end of the path
                    parent = parents[i]  # get the parent of the waypoint
                    while parent != maze.start:  # while not at start
                        path.append(parent)  # add next parent
                        parent = parents[parent]  # cycle to the next parent
                    path.append(parent)  # append the start to the path
                    path.reverse()  # reverse list so correct order
                    return path
                else:
                    explored[i] = True
                    frontier.put(i)
    return path


def astar_single(maze):
    """
    Runs A star for part 2 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path

    Manhattan Distance (Heuristic):
    abs(a.i - b.i) + abs(a.j - b.j)
    """
    path: List = []  # the path that will be returned
    frontier: List = []  # empty list with nodes ready for expansion--using heapq to make priority queue
    cost_explored = {}  # a dictionary of the explored objects
    parents = {}
    endpoint = maze.waypoints[0]

    if maze.start in maze.waypoints:
        path.append(maze.start)
        return path

    frontier.append((0, maze.start))  # tuple: (heuristic + cost, (i, j))
    cost_explored[maze.start] = 0  # abs(current[0] - endpoint[0]) + abs(current[1] - endpoint[1])

    while len(frontier) >= 0:
        heapq.heapify(frontier)
        current: List = frontier.pop(0)  # packed together: cost + heuristic, location
        curr_loc: List = current[1]

        if curr_loc in maze.waypoints:
            path.append(curr_loc)  # put waypoint at the end of the path
            parent = parents[curr_loc]
            while maze.start != parent:  # while not at start
                path.append(parent)  # add next parent
                parent = parents[parent]  # cycle to the next parent
            path.append(parent)  # append the start to the path
            path.reverse()  # reverse list so correct order
            return path

        neighbors = maze.neighbors(curr_loc[0], curr_loc[1])
        for i in neighbors:
            total_steps = cost_explored[curr_loc] + 1  # add one because the step is always 1
            if not cost_explored.get(i, False) or total_steps < cost_explored[i]:
                cost_explored[i] = total_steps
                heur_steps = total_steps + abs(curr_loc[0] - endpoint[0]) + abs(curr_loc[1] - endpoint[1])
                frontier.append((heur_steps, i))
                parents[i] = curr_loc

    return path


def astar_corner(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    path: List = []  # the path that will be returned
    frontier: List = []  # empty list with nodes ready for expansion--using heapq to make priority queue
    cost_explored = {}  # a dictionary of the explored objects
    parents = {}  # dictionary of parents
    waypoints: list[Any] = list(maze.waypoints)  # remaining waypoints
    mst = {}  # dictionary of MST's length between remaining waypoints with the remaining waypoints as the key
    wp_parent = []
    index = 0

    if maze.start in waypoints:
        waypoints.remove(maze.start)
        if len(waypoints) <= 0:
            path.append(maze.start)
            return path

    frontier.append((0, maze.start, waypoints))
    # print(frontier)
    cost_explored[maze.start] = 0

    while len(frontier) > 0:
        heapq.heapify(frontier)
        current: List = frontier.pop(0)[:]  # packed together: cost + heuristic, location, waypoints
        curr_loc: List = current[1][:]
        curr_waypoints = current[2][:]
        print(curr_waypoints)

        if curr_loc in curr_waypoints:
            curr_waypoints.remove(curr_loc)
            print(curr_waypoints)
            if len(waypoints) <= 0:
                path.append(curr_loc)  # put waypoint at the end of the path
                parent = parents[curr_loc]
                while maze.start != parent:  # while not at start
                    path.append(parent)  # add next parent
                    parent = parents[parent]  # cycle to the next parent
                path.append(parent)  # append the start to the path
                path.reverse()  # reverse list so correct order
                print(path)
                return path
            # frontier.append((current[0], current[1], curr_waypoints))

        neighbors = maze.neighbors(curr_loc[0], curr_loc[1])

        for i in neighbors:
            total_steps = cost_explored[curr_loc] + 1  # add one because the step is always 1
            if not cost_explored.get(i, False) or total_steps < cost_explored[i]:
                waypoints_key = tuple(curr_waypoints)
                if not mst.get(waypoints_key, False):
                    mst[waypoints_key] = mst_length(maze, waypoints_key)
                cost_explored[i] = total_steps
                # print(mst)
                heur_steps = total_steps + mst[waypoints_key]
                frontier.append((heur_steps, i, curr_waypoints))
                # print(curr_waypoints)
                parents[i] = curr_loc

    return path


def astar_multiple(maze):
    """
    Runs A star for part 4 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []


def fast(maze):
    """
    Runs A star for part 4 of the assignment in the case where there are
    multiple objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []


def find(parent, v):
    # print(v)
    # print(parent[v[0]])
    if v == parent[v]:
        return v
    return find(parent, parent[v])


def union(parent, rank, x1, x2):
    root1 = find(parent, x1)
    root2 = find(parent, x2)
    if rank[root1] < rank[root2]:
        parent[root1] = root2
    elif rank[root1] > rank[root2]:
        parent[root2] = root1
    else:
        parent[root2] = root1
        rank[root1] += 1


def mst_length(maze, waypoints):
    complete_graph: Dict[tuple[tuple[int, int], tuple[int, int]], int] = {}
    result = []
    parent = {}
    rank = {}
    i = 0
    e = 0

    # build complete graph
    for w1 in waypoints:
        for w2 in waypoints:
            edge_cost = astar_single_kruskals(w1, w2, maze)
            complete_graph[(w1, w2)] = edge_cost
            # print(edge_cost)

    # print(complete_graph)
    # sort complete graph by the length of the edges
    graph = list(sorted(complete_graph.items(), key=lambda x: x[1]))  # list = (w1, w2), edge_cost
    # print(graph)

    for v in waypoints:  # iterate over every object on the list
        parent[v] = v  # assign the first element to the parent
        rank[v] = 0
        # print(v)
    # print("Contents")
    # print(parent)

    while e < len(waypoints) - 1:
        v1 = graph[i][0]
        # print(v1)
        length = graph[i][1]
        i = i + 1
        x1 = find(parent, v1[0])
        x2 = find(parent, v1[1])

        if x1 != x2:
            e = e + 1
            result.append([v1[0], v1[1], length])
            union(parent, rank, x1, x2)
    return len(graph)


def astar_single_kruskals(w1, w2, maze):
    # we are looking for the length of the path
    # w1 is the starting point
    # w2 is the end point
    path: List = []
    frontier: List = []  # empty list with nodes ready for expansion--using heapq to make priority queue
    cost_explored = {}  # a dictionary of the explored objects
    parents: dict[Any, list] = {}

    if w1 == w2:
        path.append(w1)
        return len(path)

    frontier.append((0, w1))  # tuple: (heuristic + cost, (i, j))
    cost_explored[w1] = 0  # abs(current[0] - endpoint[0]) + abs(current[1] - endpoint[1])

    while len(frontier) >= 0:
        heapq.heapify(frontier)
        current: List = frontier.pop(0)  # packed together: cost + heuristic, location
        curr_loc: List = current[1]

        if curr_loc == w2:
            path.append(curr_loc)  # put waypoint at the end of the path
            parent = parents[curr_loc]
            while w1 != parent:  # while not at start
                path.append(parent)  # add next parent
                parent = parents[parent]  # cycle to the next parent
            path.append(parent)  # append the start to the path
            path.reverse()  # reverse list so correct order
            return len(path)

        neighbors = maze.neighbors(curr_loc[0], curr_loc[1])
        for i in neighbors:
            total_steps = cost_explored[curr_loc] + 1  # add one because the step is always 1
            if not cost_explored.get(i, False) or total_steps < cost_explored[i]:
                cost_explored[i] = total_steps
                heur_steps = total_steps + abs(curr_loc[0] - w2[0]) + abs(curr_loc[1] - w2[1])
                frontier.append((heur_steps, i))
                parents[i] = curr_loc

    def find(self, parent, i):
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])

    # A function that does union of two sets of x and y
    # (uses union by rank)
    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)

        # Attach smaller rank tree under root of
        # high rank tree (Union by Rank)
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot

        # If ranks are same, then make one as root
        # and increment its rank by one
        else:
            parent[yroot] = xroot
            rank[xroot] += 1
