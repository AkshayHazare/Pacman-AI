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
from heuristics import *
import random
import math

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

class RandomSequenceAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        self.actionList = [];
        for i in range(0,10):
            self.actionList.append(Directions.STOP);
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        possible = state.getAllPossibleActions();
        for i in range(0,len(self.actionList)):
            self.actionList[i] = possible[random.randint(0,len(possible)-1)];
        tempState = state;
        for i in range(0,len(self.actionList)):
            if tempState.isWin() + tempState.isLose() == 0:
                tempState = tempState.generatePacmanSuccessor(self.actionList[i]);
            else:
                break;
        # returns random action from all the valide actions
        return self.actionList[0];

class GreedyAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):
        # get all legal actions for pacman
        legal = state.getLegalPacmanActions()
        # get all the successor state for these actions
        successors = [(state.generatePacmanSuccessor(action), action) for action in legal]
        # evaluate the successor states using scoreEvaluation heuristic
        scored = [(scoreEvaluation(state), action) for state, action in successors]
        # get best choice
        bestScore = max(scored)[0]
        # get all actions that lead to the highest score
        bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
        # return random action from the list of the best actions
        return random.choice(bestActions)


class HillClimberAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return

    # GetAction Function: Called with every frame
    def getAction(self, state):
        self.root = state
        directions = self.root.getAllPossibleActions()
        score = 0
        flag = False
        main_action = Directions.STOP
        current = self.generateNew()

        while True:
            tempState = state
            for i in range(0, len(current)):
                if tempState.isWin() + tempState.isLose() == 0:
                    successor = tempState.generatePacmanSuccessor(current[i])
                    if successor == None:
                        flag = True
                        break
                    tempState = successor
                else:
                    break
            if flag == True:
                break

            if (scoreEvaluation(tempState)>score):
                score = scoreEvaluation(tempState)
                main_action = current[0]

            for index,action in enumerate(current):
                test = random.randint(0,100)
                if (test>50):
                    current[index] = random.choice(directions)

        return  main_action


    def generateNew(self):
        new = []
        directions = self.root.getAllPossibleActions()

        for i in range(0, 5):
            new.append(random.choice(directions))
        return new


class GeneticAgent(Agent):
    # Initialization Function: Called one time when the game starts
    def registerInitialState(self, state):
        return Directions.STOP

    # GetAction Function: Called with every frame
    def getAction(self, state):
        self.root = state

        directions = self.root.getAllPossibleActions()
        generation = []
        for i in range(0,8):
            generation.append(self.generateChromosome())

        new_score = []
        flag = False
        while True:
            score = []
            for child in generation:
                tempState = state
                for i in range(0,len(child)):

                    if tempState.isWin() + tempState.isLose() == 0:
                        successor = tempState.generatePacmanSuccessor(child[i])
                        if successor == None:
                            flag = True
                            break
                        tempState = successor
                    else:
                        break
                if flag == True:
                    break
                score.append((scoreEvaluation(tempState),child))

            if flag == True:
                break
            score.sort(key=lambda x: x[0])
            new_score = score
            a = []
            for i in range(0,8):
                for j in range(0,i+1):
                        a.append(j+1)

            score1,p1 = score[random.choice(a)-1]
            score2,p2 = score[random.choice(a)-1]

            test1 = random.randint(0,100)
            if (test1<=70):
                c1,c2 = self.crossOver(p1,p2)
                for index,child in enumerate(generation):
                    if child == p1 :
                        generation[index] = c1
                    if child == p2:
                        generation[index] = c2

            for child in generation:
                test3 = random.randint(0,100)
                if (test3 <= 10):
                    child[random.randint(0,4)]=random.choice(directions)

        scores,path = new_score.pop()
        return path[0]

    def generateChromosome(self):
        chromosome = []
        directions = self.root.getAllPossibleActions()
        for i in range(0,5):
            chromosome.append(random.choice(directions))
        return chromosome

    def crossOver(self,p1,p2):
        c = []
        for i in (0,2):
            x = []
            for j in range(0,len(p1)):
                test = random.randint(0,100)
                if(test<50):
                    x.append(p1[j])
                else:
                    x.append(p2[j])
            c.append(x)
        return c[0],c[1]



class MCTSAgent(Agent):
    # Initialization Function: Called one time when the game starts

    def registerInitialState(self, state):
        return;

    # GetAction Function: Called with every frame
    def getAction(self, state):


        ## Tree class to make the Tree
        class Tree_Node():
            def __init__(self,action=None,parent=None):
                self.visits = 0
                self.reward = 0.0
                self.action = action
                self.children = []
                self.parent = parent

            ### Add a child to the given node at the given action
            def add_child(self, action):
                child = Tree_Node(action,self)
                self.children.append(child)

            ### get_child - to get the child of the current node from the given action
            def get_child(self,action):
                for child in self.children:
                    if child.action == action:
                        return child

            ### Back propagation of reward and visits till the Root
            def back_prop(node, reward):
                while node != None:
                    node.visits += 1
                    node.reward += reward
                    node = node.parent
                return

        self.root = state
        Root = Tree_Node()
        Root.add_child(None)

        None_Flag = False
        while True:
            if None_Flag == True: break
            current_node = Root
            current_state = self.root
            while True:
                Start_from_root = False
                actions = current_state.getLegalPacmanActions()
                UCT = 0
                chosen_action = Directions.STOP
                for action in actions:
                    temp_term = current_state.generatePacmanSuccessor(action)
                    if temp_term == None:
                        None_Flag = True,
                        break
                    if (temp_term.isWin() + temp_term.isLose() == 0):
                        cur = current_node.get_child(action)
                        if cur == None:
                            current_node.add_child(action)
                            current_node.get_child(action).back_prop(self.Rollout(temp_term))
                            Start_from_root = True
                            break
                        else:
                            temp = self.UCT(cur)
                            if (temp>UCT):
                                UCT = temp
                                chosen_action = action
                    else:
                        chosen_action = Directions.STOP
                        continue

                if Start_from_root== True: break
                if None_Flag == True: break
                current_state = current_state.generatePacmanSuccessor(chosen_action)
                if current_state == None:
                    None_Flag = True
                    break
                current_node = current_node.get_child(chosen_action)
                if current_node == None: break

        temp = 0
        action = Directions.STOP
        for node in Root.children:
            if node.visits>temp:
                action = node.action
        return action

    def UCT(self, node):
        value = node.reward / float(node.visits) + math.sqrt(2 * math.log(node.parent.visits) / float(node.visits))
        return value

    def Rollout(self,cur):
        current = cur
        for i in range(0, 5):
            if (current.isLose() + current.isWin() != 0):
                return normalizedScoreEvaluation(self.root, current)
            else:
                action = random.choice(current.getAllPossibleActions())
                successor = current.generatePacmanSuccessor(action)
                if successor == None: break
                current = successor
        return normalizedScoreEvaluation(self.root, current)

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


