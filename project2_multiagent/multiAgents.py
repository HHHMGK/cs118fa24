# multiAgents.py
# --------------
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


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent
from pacman import GameState

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """


    def getAction(self, gameState: GameState):
        """
        You do not need to change this method, but you're welcome to.

        getAction chooses among the best options according to the evaluation function.

        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {NORTH, SOUTH, WEST, EAST, STOP}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState: GameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState: GameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"
        depth = 0
        maxValue = float("-inf")
        maxAction = None
        for action in gameState.getLegalActions(0):
            actionValue = self.value(gameState.generateSuccessor(0, action), depth, 1)
            if (maxValue < actionValue):
                maxValue = actionValue
                maxAction = action
        return maxAction
    def value(self, gameState: GameState, depth: int, agentIndex: int):
        if gameState.isWin() or gameState.isLose() or depth == self.depth:
            return self.evaluationFunction(gameState)
        if agentIndex == 0:
            return self.maxValue(gameState, depth)
        else:
            return self.minValue(gameState, depth, agentIndex)

    def maxValue(self, gameState: GameState, depth: int):
        v = float("-inf")
        for action in gameState.getLegalActions(0):
            v = max(v, self.value(gameState.generateSuccessor(0, action), depth, 1))
        return v

    def minValue(self, gameState: GameState, depth: int, agentIndex: int):
        v = float("inf")
        for action in gameState.getLegalActions(agentIndex):
            if agentIndex == gameState.getNumAgents() - 1:
                v = min(v, self.value(gameState.generateSuccessor(agentIndex, action), depth + 1, 0))
            else:
                v = min(v, self.value(gameState.generateSuccessor(agentIndex, action), depth, agentIndex + 1))
        return v

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        alpha = float("-inf")
        beta = float("inf")
        depth = 0
        maxValue = float("-inf")
        maxAction = 0
        for action in gameState.getLegalActions(0):
            actionValue = self.value(gameState.generateSuccessor(0, action), alpha, beta, depth, 1)
            if (maxValue < actionValue):
                maxValue = actionValue
                maxAction = action
            alpha = max(actionValue, alpha)
        return maxAction

    # This function returns the value of the gameState
    def value(self, gameState: GameState, alpha: int, beta: int, depth: int, agentIndex: int):
        # if the game is won or lost or the depth is reached return the value of the evaluation function for the gameState
        if gameState.isWin() or gameState.isLose() or depth == self.depth:
            return self.evaluationFunction(gameState)
        if agentIndex == 0: # if the agent is Pacman calculate the maximum value
            return self.maxValue(gameState, alpha, beta, depth)
        else: # if the agent is a ghost calculate the minimum value
            return self.minValue(gameState, alpha, beta, depth, agentIndex)

    # This function returns the maximum value for Pacman
    # It prunes branches where the value is greater than beta
    def maxValue(self, gameState: GameState, alpha: int, beta: int, depth: int):
        v = float("-inf") # initialize v to negative infinity
        # for each action in legal actions of pacman
        for action in gameState.getLegalActions(0):
            # calculate the value of its child nodes
            child_value = self.value(gameState.generateSuccessor(0, action), alpha, beta, depth, 1)
            v = max(v, child_value)
            if v > beta: # prune the tree if v is greater than beta
                return v
            alpha = max(alpha, v)
        return v

    # This function returns the minimum value for the ghosts
    # It prunes branches where the value is less than alpha
    def minValue(self, gameState: GameState, alpha: int, beta: int, depth: int, agentIndex: int):
        v = float("inf") # initialize v to positive infinity
        # for each action in legal actions of the ghost
        for action in gameState.getLegalActions(agentIndex):
            # if the agent is the last ghost, increment the depth and set the agentIndex to 0 (Pacman)
            if agentIndex == gameState.getNumAgents() - 1:
                child_value = self.value(gameState.generateSuccessor(agentIndex, action), alpha, beta, depth + 1, 0)
                v = min(v, child_value)
            else: # else increment the agentIndex (next ghost)
                child_value = self.value(gameState.generateSuccessor(agentIndex, action), alpha, beta, depth, agentIndex + 1)
                v = min(v, child_value)
            if v < alpha: # prune the tree if v is less than alpha
                return v
            beta = min(v, beta)
        return v





class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState: GameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        depth = 0
        maxValue = float("-inf")
        maxAction = None
        for action in gameState.getLegalActions(0):
            actionValue = self.value(gameState.generateSuccessor(0, action), depth, 1)
            if (maxValue < actionValue):
                maxValue = actionValue
                maxAction = action
        return maxAction

    def value(self, gameState: GameState, depth: int, agentIndex: int):
        if gameState.isWin() or gameState.isLose() or depth == self.depth:
            return self.evaluationFunction(gameState)
        if agentIndex == 0:
            return self.maxValue(gameState, depth)
        else:
            return self.expValue(gameState, depth, agentIndex)

    # This function returns the maximum value from the child nodes of Pacman moves
    def maxValue(self, gameState: GameState, depth: int):
        v = float("-inf")
        for action in gameState.getLegalActions(0):
            child_value = self.value(gameState.generateSuccessor(0, action), depth, 1)
            v = max(v, child_value)
        return v

    # This function returns the expected (average) value from the child nodes of the ghost moves
    def expValue(self, gameState: GameState, depth: int, agentIndex: int):
        v = 0 # this variable will store the sum of the values of the child nodes
        for action in gameState.getLegalActions(agentIndex):
            # if the agent is the last ghost, increment the depth and set the agentIndex to 0 (Pacman)
            if agentIndex == gameState.getNumAgents() - 1:
                v += self.value(gameState.generateSuccessor(agentIndex, action), depth + 1, 0)
            else: # else increment the agentIndex (next ghost)
                v += self.value(gameState.generateSuccessor(agentIndex, action), depth, agentIndex + 1)
        # return the average value of the child nodes
        return v / len(gameState.getLegalActions(agentIndex))

def betterEvaluationFunction(currentGameState: GameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    # ghostScareTimes = [ghostState.scaredTimer for ghostState in currentGameState.getGhostStates()]
    # avgGhostScareTime = sum(ghostScareTimes) / len(ghostScareTimes)
    # return currentGameState.getScore() + avgGhostScareTime * 100
    foodList = currentGameState.getFood().asList()
    pacmanPos = currentGameState.getPacmanPosition()
    distToFood = [manhattanDistance(pacmanPos, food) for food in foodList]
    if len(distToFood) == 0:
        distToFood.append(0)
    ghostPositions = currentGameState.getGhostPositions()
    distToGhost = [manhattanDistance(pacmanPos, ghost) for ghost in ghostPositions]
    if len(distToGhost) == 0:
        distToGhost.append(0)
    # scaring = [ghost.scaredTimer for ghost in currentGameState.getGhostStates()]
    ghostScore = []
    for ghost in currentGameState.getGhostStates():
        dist = manhattanDistance(pacmanPos, ghost.getPosition())
        if ghost.scaredTimer == 0:
            ghostScore.append(dist)
        else:
            ghostScore.append(-dist)
    return currentGameState.getScore() - (sum(distToFood) / len(distToFood)) - min(ghostScore)

# Abbreviation
better = betterEvaluationFunction
