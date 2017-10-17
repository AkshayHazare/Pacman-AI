# pacmanAgents.py
# ---------------
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


from pacman import Directions
from game import Agent
from heuristics import scoreEvaluation
import random


class RandomAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        actions = state.getLegalPacmanActions()
        # returns random action from all the valide actions
        return actions[random.randint(0,len(actions)-1)]

class GreedyAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = [(state.generateSuccessor(0, action), action) for action in legal]
        # evaluate the successor states using scoreEvaluation heuristic
        scored = [(scoreEvaluation(state), action) for state, action in successors]
        # get best choice
        bestScore = max(scored)[0]
        # get all actions that lead to the highest score
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        # return random action from the list of the best actions
        return random.choice(bestActions)

### Implementing BFS, DFS and AStar
class BFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        queue = []
        flag = False
        legal = state.getLegalPacmanActions()
        depth = 1
        for action in legal:
            path = state.generatePacmanSuccessor(action)
            queue.append((path, depth, action))
        while queue:
            temp = queue.pop(0)
            current, depth, action = temp
            legal = current.getLegalPacmanActions()
            if (current.isWin()):
                return action
            if (current.isLose()):
                continue
            for next in legal:
                successor = current.generatePacmanSuccessor(next)
                if (successor == None):
                    flag = True
                    break
                else:
                    queue.append((successor, depth+1, action))
            if(flag):
                break

        bestAction = Directions.STOP
        scored = [(scoreEvaluation(states), depth, action) for states, depth, action in queue]
        if (scored != None):
        ## Finding the maximum score
            bestScore = max(scored)[0]
            scored1 = [(score, depth, action) for score, depth, action in scored if score == bestScore]
            bestAction = min(scored1, key=lambda item: item[1])[2]      ## Finding the action corresponding to the min depth
        return bestAction


class DFSAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        queue = []
        flag = False
        legal = state.getLegalPacmanActions()
        depth = 1
        for action in legal:
            path = state.generatePacmanSuccessor(action)
            queue.append((path, depth, action))
        while queue:
            temp = queue.pop()
            current, depth, action = temp
            legal = current.getLegalPacmanActions()
            if (current.isWin()):
                return action
            if (current.isLose()):
                continue
            for next in legal:
                successor = current.generatePacmanSuccessor(next)
                if (successor == None):
                    flag = True
                    break
                else:
                    queue.append((successor, depth + 1, action))
            if (flag):
               break

        bestAction = Directions.STOP
        scored = [(scoreEvaluation(states), depth, action) for  states, depth, action in queue]
        if (scored != None):
        ## Finding the maximum score
            bestScore = max(scored)[0]
            scored1 = [(score, depth, action) for score, depth, action in scored if score == bestScore]
            bestAction = min(scored1, key=lambda item: item[1])[2]   ## Finding the action corresponding to the min depth
        return bestAction


class AStarAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        flag = False  # used to set the flag when None type is returned from getPacmanSuccessor
        successors = []
        legal = state.getLegalPacmanActions()
        depth = 1
        for action in legal:
            path = state.generatePacmanSuccessor(action)
            cost = depth - (scoreEvaluation(path) - scoreEvaluation(state))
            successors.append((cost, path, action, depth))
        while (successors):
            if (flag):
                break
            successors.sort()
            cost, cur, action, depth = successors.pop(0)
            if (cur.isWin()):
                return action
            if (cur.isLose()):
                continue
            legal = cur.getLegalPacmanActions()
            for next in legal:
                successor = cur.generatePacmanSuccessor(next)
                if (successor == None):
                    flag = True
                    break
                cost = (depth + 1) - (scoreEvaluation(successor) - scoreEvaluation(state))
                successors.append((cost, successor, action, depth + 1))

        # If no terminal state, return the action leading to the node with
        # the best score and no children based on the heuristic function (scoreEvaluation)
        bestAction = Directions.STOP
        scored = [(scoreEvaluation(states), depth, action) for cost, states, action, depth in successors]
        ## Finding the maximum score
        if (scored != None):
            bestScore = max(scored)[0]
            scored1 = [(score, depth, action) for score, depth, action in scored if score == bestScore]
            bestAction = min(scored1, key=lambda item: item[1])[2]  ## Finding the action corresponding to the min depth
        return bestAction