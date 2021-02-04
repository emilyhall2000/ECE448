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
from typing import Any, List
import heapq


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
    endpoint = maze.waypoint[0]

    if maze.start in maze.waypoints:
        path.append(maze.start)
        return path

    frontier.append((0, maze.start)) # tuple: (heuristic + cost, (i, j))
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

        next: List = frontier.pop(1)  # packed together: cost + heuristic, location
        next_loc: List = current[1]
        neighbors = maze.neighbors(curr_loc[0], curr_loc[1])

        for i in neighbors:
            total_steps = cost_explored[curr_loc] + 1  # add one because the step is always 1
            if not cost_explored.get(i, False) or total_steps < cost_explored[next_loc]:
                cost_explored[next_loc] = total_steps
                heur_steps = total_steps + abs(current[0] - endpoint[0]) + abs(current[1] - endpoint[1])
                frontier.append( heur_steps, i)
                parents[i] = curr_loc

    return path

def astar_corner(maze):
    """
    Runs A star for part 3 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """

    return []

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
    Runs suboptimal search algorithm for part 5.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    return []
    
            
