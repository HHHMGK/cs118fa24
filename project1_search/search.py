# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    
    stack = util.Stack()
    prev = {}
    start = problem.getStartState()
    goal = None
    stack.push((start, None, None))
    while not stack.isEmpty():
        state, prev_state, prev_act = stack.pop()
        # print(state, prev_state, prev_act)
        if state in prev:
            continue
        prev[state] = [prev_state, prev_act]
        if problem.isGoalState(state):
            goal = state
            break
        for successor in problem.getSuccessors(state):
            stack.push((successor[0], state, successor[1]))
    
    actions = []
    while goal != start:
        actions.append(prev[goal][1])
        goal = prev[goal][0]
    actions.reverse()
    return actions

def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    queue = util.Queue()
    prev = {}
    start = problem.getStartState()
    goal = None
    queue.push(start)
    prev[problem.getStartState()] = [(None, None), None]
    while not queue.isEmpty():
        state = queue.pop()
        if problem.isGoalState(state):
            goal = state
            break
        for successor in problem.getSuccessors(state):
            if successor[0] in prev:
                continue
            queue.push(successor[0])
            prev[successor[0]] = [state, successor[1]]
    
    actions = []
    while goal != start:
        actions.append(prev[goal][1])
        goal = prev[goal][0]
    actions.reverse()
    return actions

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    # print("Successors:", problem.getSuccessors(problem.getStartState()))
    pqueue = util.PriorityQueue()
    prev = {}
    start = problem.getStartState()
    goal = None
    pqueue.push((start, None, None, 0), 0)
    while not pqueue.isEmpty():
        state, prev_state, prev_act, path_len = pqueue.pop()
        if state in prev:
            continue
        prev[state] = [prev_state, prev_act]
        if problem.isGoalState(state):
            goal = state
            break
        for successor in problem.getSuccessors(state):
            pqueue.push((successor[0], state, successor[1], path_len + successor[2]), path_len + successor[2])
    
    actions = []
    while goal != start:
        actions.append(prev[goal][1])
        goal = prev[goal][0]
    actions.reverse()
    return actions

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    pqueue = util.PriorityQueue()
    prev = {}
    gcost = {} # Path cost
    start = problem.getStartState()
    goal = None
    pqueue.push((start, None, None, heuristic(start, problem)), heuristic(start, problem))
    while not pqueue.isEmpty():
        state, prev_state, prev_act, path_len = pqueue.pop()
        # if state in prev:
        #     continue
        if state in gcost and gcost[state] <= path_len:
            continue
        prev[state] = [prev_state, prev_act]
        gcost[state] = path_len
        if problem.isGoalState(state):
            goal = state
            break
        for successor in problem.getSuccessors(state):
            path_cost = path_len + successor[2]
            heu_cost = heuristic(successor[0], problem)
            if successor[0] in gcost and gcost[successor[0]] <= path_cost:
                continue
            pqueue.push((successor[0], state, successor[1], path_cost), path_cost + heu_cost)
    
    actions = []
    while goal != start:
        actions.append(prev[goal][1])
        goal = prev[goal][0]
    actions.reverse()
    return actions
    

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
