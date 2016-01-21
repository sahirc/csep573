# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
import string

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s,s,w,s,w,w,s,w]

def depthFirstSearch(problem):
    return graphSearch(problem, util.Stack())

def breadthFirstSearch(problem):
    return graphSearch(problem, util.Queue())

def graphSearch(problem, struct):
    nodes = struct
    nodes.push((problem.getStartState(), []))

    visited = []
    processedSucc = set()
    while nodes:
        state = nodes.pop()
        vertex, directionsSoFar = state
        if (problem.isGoalState(vertex)):
            return directionsSoFar
        else:
            if vertex not in visited:
                nextSucc = problem.getSuccessors(vertex)
                visited.append(vertex)
                for coord, direction, cost in nextSucc:
                    if coord not in visited:
                        newDir = directionsSoFar + [direction]
                        nodes.push((coord, newDir))    
    
    

def uniformCostSearch(problem):
    # UCS is A* with a trivial heuristic
    return aStarSearch(problem, nullHeuristic)

def nullHeuristic(state, problem=None):
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    queue = util.PriorityQueue()
    visited = []
    queue.push((problem.getStartState(), []), 0)

    while queue:
        vertex, directionsSoFar = queue.pop()
        if (problem.isGoalState(vertex)):
            return directionsSoFar
        else:
            if vertex not in visited:
                nextSucc = problem.getSuccessors(vertex)
                visited.append(vertex)                   
                for coord, direction, cost in nextSucc:
                    if coord not in visited:
                        newDir = directionsSoFar + [direction]
                        queue.push((coord, newDir), problem.getCostOfActions(newDir) + heuristic(coord, problem))

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch