# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    return search(problem, util.Stack())

def breadthFirstSearch(problem):
    return search(problem, util.Queue())

def search(problem, structure):                    
    nodes = structure
    nodes.push((problem.getStartState(), []))

    visited = set()
    while nodes:
        vertex, directionsSoFar = nodes.pop()
        if (problem.isGoalState(vertex)):
            return directionsSoFar
        else:
            visited.add(vertex)        
            nextSucc = problem.getSuccessors(vertex)
            for coord, direction, cost in nextSucc:
                if coord not in visited:
                    newDir = directionsSoFar + [direction]
                    nodes.push((coord, newDir))


def uniformCostSearch(problem):
    queue = util.PriorityQueue()
    visited = set()
    queue.push((problem.getStartState(), []), 0)

    while queue:
        vertex, directionsSoFar = queue.pop()
        visited.add(vertex)        
        if (problem.isGoalState(vertex)):
            return directionsSoFar
        else:
            nextSucc = problem.getSuccessors(vertex)
            for coord, direction, cost in nextSucc:
                if coord not in visited:
                    newDir = directionsSoFar + [direction]
                    queue.push((coord, newDir), problem.getCostOfActions(newDir))


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    queue = util.PriorityQueue()
    visited = set()
    queue.push((problem.getStartState(), []), 0)

    while queue:
        vertex, directionsSoFar = queue.pop()
        visited.add(vertex)        
        if (problem.isGoalState(vertex)):
            return directionsSoFar
        else:
            nextSucc = problem.getSuccessors(vertex)
            for coord, direction, cost in nextSucc:
                if coord not in visited:
                    newDir = directionsSoFar + [direction]
                    queue.push((coord, newDir), problem.getCostOfActions(newDir) + heuristic(coord, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
