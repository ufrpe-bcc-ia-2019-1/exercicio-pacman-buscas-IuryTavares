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

#busca em profundidade
def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.
    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    print "Start:", problem.getStartState() ============(5,5)
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())   ============True
    print "Start's successors:", problem.getSuccessors(problem.getStartState())  ===========[((x1,y1),'South',1),((x2,y2),'West',1)]
    """
    "*** YOUR CODE HERE ***"

    #inicializacao
    fringe = util.Stack()
    visitedList = []

    #coloque o ponto de partida para a fila
    fringe.push((problem.getStartState(),[],0))
    #estendendo o ponto
    (state,toDirection,toCost) = fringe.pop()
    #coloque o ponto visitado na lista de pontos visitados
    visitedList.append(state)

    while not problem.isGoalState(state): #enquanto nao enconttramos o caminho onjetivo
        successors = problem.getSuccessors(state) #pegue os pontos sucessoress
        for son in successors:
            if (not son[0] in visitedList) or (problem.isGoalState(son[0])): #caso o ponto sucessor n tenha sido visitado coloca na pilha
                fringe.push((son[0],toDirection + [son[1]],toCost + son[2]))
                visitedList.append(son[0]) #coloque o ponto visitado na lista depontos visitados
        (state,toDirection,toCost) = fringe.pop()

    return toDirection

#busca em largura
def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    #inicializacao
    fringe = util.Queue()
    visitedList = []

    #coloque o ponto de partida para a fila
    fringe.push((problem.getStartState(),[],0))
    #estenddo o ponto
    (state,toDirection,toCost) = fringe.pop()
    #coloque o ponto visitado na lista de pontos visitados
    visitedList.append(state)

    while not problem.isGoalState(state): #enquanto n encontramos o caminho objetivo
        successors = problem.getSuccessors(state) #pegue os pontos sucessores
        for son in successors:
            if not son[0] in visitedList: #se o ponto sucessor n tive sido visitado, coloque na fila
                fringe.push((son[0],toDirection + [son[1]],toCost + son[2]))
                visitedList.append(son[0]) # add esse ponto a lista visitada
        (state,toDirection,toCost) = fringe.pop()

    return toDirection

#busca de custo uniforme
def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    from game import Directions

    #iniciando
    fringe = util.PriorityQueue()
    visitedList = []

    #coloque o ponto de partida para a fila
    fringe.push((problem.getStartState(),[],0),0) # push starting point with priority num of 0
    #estenda o ponto
    (state,toDirection,toCost) = fringe.pop()
    #coloque o ponto na lista de ponto visitados
    visitedList.append((state,toCost))

    while not problem.isGoalState(state): #enquanto n encontramos o caminho objetivo
        successors = problem.getSuccessors(state) #pegue os pontos sucessores
        for son in successors:
            visitedExist = False
            total_cost = toCost + son[2]
            for (visitedState,visitedToCost) in visitedList:
                #adicione o ponto se o sucessor n foi visitado ou foi visitado, mas agr com um custo menor que o anterir
                if (son[0] == visitedState) and (total_cost >= visitedToCost):
                    visitedExist = True # ponto reconhecido visitado
                    break

            if not visitedExist:
                # empurre o ponto com o num de prioriade do seu custo total
                fringe.push((son[0],toDirection + [son[1]],toCost + son[2]),toCost + son[2])
                visitedList.append((son[0],toCost + son[2])) # coloque o ponto na lista de pontos visistados

        (state,toDirection,toCost) = fringe.pop()

    return toDirection

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"


    #iniciando
    fringe = util.PriorityQueue()
    visitedList = []

    #coloque o ponto de partida para a fila
    fringe.push((problem.getStartState(),[],0),0 + heuristic(problem.getStartState(),problem)) # push starting point with priority num of 0
    #estenda o ponto
    (state,toDirection,toCost) = fringe.pop()
    #add the point to visited list
    visitedList.append((state,toCost + heuristic(problem.getStartState(),problem)))

    while not problem.isGoalState(state): #enquanto n encontramos o caminho
        successors = problem.getSuccessors(state) #pegue o pontos sucessores
        for son in successors:
            visitedExist = False
            total_cost = toCost + son[2]
            for (visitedState,visitedToCost) in visitedList:
                # se o sucessor n tiver sisdo visitado ou tiver custo menor que o anterior
                if (son[0] == visitedState) and (total_cost >= visitedToCost):
                    visitedExist = True
                    break

            if not visitedExist:
                # empurre o ponto com o num de prioridade do seu custo total
                fringe.push((son[0],toDirection + [son[1]],toCost + son[2]),toCost + son[2] + heuristic(son[0],problem))
                visitedList.append((son[0],toCost + son[2])) #adicione o ponto a lista de pntos visitados

        (state,toDirection,toCost) = fringe.pop()

    return toDirection


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
