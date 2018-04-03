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

#ADDED Node, node2 class and EXPAND FUNCTION
class node:
	def __init__(self, state, depth, path_actions, path_cost):
	    self.state = state
	    self.depth = depth
	    self.path_actions = path_actions
	    self.path_cost = path_cost

#def expand(node):
#    succ = 

class node2(node):
    "Node with evaluation function added"
    def __init__(self, state, depth, path_actions, path_cost, f):
        self.f = f
        node.__init__(self, state, depth, path_actions, path_cost)

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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    #"*** YOUR CODE HERE ***"

    """
    Pseudocode:
        function G RAPH-S EARCH ( problem) returns a solution, or failure
            initialize the frontier using the initial state of problem
            initialize the explored set to be empty
            loop do
                if the frontier is empty then return failure
                    choose a leaf node and remove it from the frontier
                if the node contains a goal state then return the corresponding solution
                add the node to the explored set
                expand the chosen node, adding the resulting nodes to the frontier
                    only if not in the frontier or explored set

    """
    frontier = util.Stack()
    #print 'Create frontier'
    initial_node = node(problem.getStartState(), 0, [], 0)#(state,depth,path_actions,path_cost)
    frontier.push(initial_node)
    #print 'Push ',repr(initial_node.state)
    frontierSet = set([initial_node.state])
    explored = set() #initialize the explored set to be empty

    while True:
        if frontier.isEmpty() == True: raise Exception, "The frontier was emptied"#if the frontier is empty then return failure
        currNode = frontier.pop()#HERE1
        frontierSet.remove(currNode.state)
        #print 'Remove',repr(currNode.state)
        #print 'State: ' + repr(currNode.state) + '. Depth: ' + repr(currNode.depth) + '. Path Cost: ' + repr(currNode.path_cost) + '. Path Actions: ' + repr(currNode.path_actions) + '.\n'
        if problem.isGoalState(currNode.state) == True:
            print 'Goal reached!'
            return currNode.path_actions
        explored.add(currNode.state)
        for succ in problem.getSuccessors(currNode.state):
            #print 'Succ: ',repr(succ[0])
            succNode = node(succ[0], currNode.depth + 1, currNode.path_actions + [succ[1],], currNode.path_cost + succ[2])
            if (succNode.state not in explored):
                # Si hacemos estas verificaciones entonces cuando se encuentra que un estado que se quiere expandir ya esta en la frontera
                # eliminamos ese estado de la frontera y lo expandimos ahora. Osea, damos prioridad a los nodos nuevos
                if(succNode.state in frontierSet):
                    # Recurso'i:
                    for frontierNode in frontier.list:
                        if frontierNode.state == succNode.state:
                            frontier.list.remove(frontierNode)
                            frontierSet.remove(frontierNode.state)
            # if ((succNode.state not in explored) and (succNode.state not in frontierSet)): 
            # Alternativa segun el libro. Lo que se hace es que se da prioridad a los nodos viejos.

            # Aca no verificaba si ya esta en la frontera porque alteraba el orden en el que se visitan los nodos.
            # Por ejemplo cuando esta pendiente (se genero pero no se expandio) un hijo con un estado,
            # pero en un nivel mas profundo se vuelve a generar el mismo estado y se tiene que expandir.
            # Si seguimos el DFS creo que tendriamos que expandir ese nodo ahi y no en la primera llamada donde quedo pendiente.
                
                frontier.push(succNode)
                #print 'Push ',repr(succNode.state)
                frontierSet.add(succNode.state)

    #util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first.
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
    """

    frontier = util.Queue()
    # print 'Create frontier'
    initial_node = node(problem.getStartState(), 0, [], 0)#(state,depth,path_actions,path_cost)
    frontier.push(initial_node)
    # print 'Push ',repr(initial_node.state)
    frontierSet = set([initial_node.state])
    explored = set() #initialize the explored set to be empty

    while True:
        if frontier.isEmpty() == True: raise Exception, "The frontier was emptied"#if the frontier is empty then return failure
        currNode = frontier.pop()#HERE1
        frontierSet.remove(currNode.state)
        # print 'Remove',repr(currNode.state)
        # print 'State: ' + repr(currNode.state) + '. Depth: ' + repr(currNode.depth) + '. Path Cost: ' + repr(currNode.path_cost) + '. Path Actions: ' + repr(currNode.path_actions) + '.\n'
        if problem.isGoalState(currNode.state) == True:
            print 'Goal reached!'
            return currNode.path_actions
        explored.add(currNode.state)
        for succ in problem.getSuccessors(currNode.state):
            # print 'Succ: ',repr(succ[0])
            succNode = node(succ[0], currNode.depth + 1, currNode.path_actions + [succ[1],], currNode.path_cost + succ[2])
            if (succNode.state not in explored) and (succNode.state not in frontierSet):
                """Aca si hay que verificar si es que ya esta en la frontera porque es formato FIFO. Entonces los nodos que estan en la lista
                necesariamente van a ser verificados antes de que se vuelva a insertar otro.
                """
                frontier.push(succNode)
                # print 'Push ',repr(succNode.state)
                frontierSet.add(succNode.state)

def uniformCostSearch(problem):
    """Search the node of least total cost first.
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()
    """

    frontier = util.PriorityQueue()
    #print 'Create frontier'
    initial_node = node(problem.getStartState(), 0, [], 0)#(state,depth,path_actions,path_cost)
    frontier.push(initial_node, initial_node.path_cost)
    #print 'Push ',repr(initial_node.state)
    frontierSet = set([(initial_node.state, initial_node.path_cost)])
    explored = set() #initialize the explored set to be empty

    while True:
        if frontier.isEmpty() == True: raise Exception, "The frontier was emptied"#if the frontier is empty then return failure
        currNode = frontier.pop()#HERE1
        frontierSet.remove((currNode.state, currNode.path_cost))
        #print 'Remove',repr(currNode.state)
        #print 'State: ' + repr(currNode.state) + '. Depth: ' + repr(currNode.depth) + '. Path Cost: ' + repr(currNode.path_cost) + '. Path Actions: ' + repr(currNode.path_actions) + '.\n'
        if problem.isGoalState(currNode.state) == True:
            print 'Goal reached!'
            return currNode.path_actions
        explored.add(currNode.state)
        for succ in problem.getSuccessors(currNode.state):
            #print 'Succ: ',repr(succ[0])
            succNode = node(succ[0], currNode.depth + 1, currNode.path_actions + [succ[1],], currNode.path_cost + succ[2])
            if (succNode.state not in explored):
                """Aca si hay que verificar si es que ya esta en la frontera porque es formato FIFO. Entonces los nodos que estan en la lista necesariamente van a ser
                verificados antes de que se vuelva a insertar otro, cumpliendo con el algoritmo.
                """

                StateInFrontierSet = False
                ExistsBetterPriority = False
                for frontierSet_node in frontierSet:
                    if (succNode.state == frontierSet_node[0]):
                        StateInFrontierSet = True
                        if (succNode.path_cost < frontierSet_node[1]):
                            ExistsBetterPriority = True
                            frontierSet.remove(frontierSet_node)
                            #print 'Remove ',repr((frontierSet_node[0], frontierSet_node[1]))

                            #Recurso'i:
                            for prio, count, frontierNode in frontier.heap:
                                if frontierNode.state == succNode.state:
                                    frontier.heap.remove((prio, count, frontierNode))
                            """
                            Recurso'i. Hay que cambiar la estructura de los nodos para que contenga solo el action_cost, en lugar del path_cost
                            y para guardar la solucion tener una estructura aparte a la que se le van appendeando las acciones,
                            o capaz seguir la implementacion del libro y hacer una funcion con el nodo como parametro y calcula la solucion,
                            o hacer que frontier solo tenga los estados?
                            frontier.update(succNode, succNode.path_cost) con esta operacion deberia de bastar
                            """
                            break
                            
                if not (StateInFrontierSet and not ExistsBetterPriority): # El caso en que no se hace nada es cuando ya esta en la frontera
                # pero con una mejor o igual prioridad
                    frontier.push(succNode, succNode.path_cost)
                    #print 'Push ',repr((succNode.state, succNode.path_cost))
                    frontierSet.add((succNode.state, succNode.path_cost))



def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def evaluationFunction(problem, gFunc, hFunc, node):
    """Recibe dos funciones, g y h, y calcula la suma de g(node) + h(node)"""
    #g = getattr(searchAgents, gFunc)
    #h = getattr(searchAgents, hFunc)
    h = hFunc
    #return g(node) + h(node)
    return gFunc + h(node, problem)

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first.
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()"""

    frontier = util.PriorityQueue()
    #print 'Create frontier'
    initial_state = problem.getStartState()
    initial_node = node2(initial_state, 0, [], 0 , evaluationFunction(problem, 0, heuristic, initial_state))#(state,depth,path_actions,path_cost,f)

    frontier.push(initial_node, initial_node.f)
    #print 'Push ',repr((initial_node.state, initial_node.f))
    frontierSet = set([(initial_node.state, initial_node.f)])
    explored = set() #initialize the explored set to be empty

    while True:
        if frontier.isEmpty() == True: raise Exception, "The frontier was emptied"#if the frontier is empty then return failure
        currNode = frontier.pop()#HERE1
        frontierSet.remove((currNode.state, currNode.f))
        #print 'Remove',repr((currNode.state, currNode.f))
        #print 'State: ' + repr(currNode.state) + '. Depth: ' + repr(currNode.depth) + '. Path Cost: ' + repr(currNode.path_cost) + '. EvalFunc: ' + repr(currNode.f) + '. Path Actions: ' + repr(currNode.path_actions) + '.\n'
        if problem.isGoalState(currNode.state) == True:
            print 'Goal reached!'
            return currNode.path_actions
        explored.add(currNode.state)
        for succ in problem.getSuccessors(currNode.state):
            succState = succ[0]
            succAction = succ[1]
            succActionCost = succ[2]

            #print 'Succ: ',repr((succState, succAction, succActionCost))
            succEvalFunc = evaluationFunction(problem, currNode.path_cost + succActionCost, heuristic, succState)
            #print 'State: %s. Heuristic : %s. h = %s. g = %s. f = %s' % (succState, repr(heuristic), heuristic(succState, problem), currNode.path_cost + succActionCost , succEvalFunc)
            succNode = node2(succState, currNode.depth + 1, currNode.path_actions + [succAction,], currNode.path_cost + succActionCost, succEvalFunc)
            if (succNode.state not in explored):
                """Aca si hay que verificar si es que ya esta en la frontera porque es formato FIFO.
                Entonces los nodos que estan en la lista necesariamente van a ser
                verificados antes de que se vuelva a insertar otro, cumpliendo con el algoritmo.
                """

                StateInFrontierSet = False
                ExistsBetterPriority = False
                for frontierSet_node in frontierSet:
                    if (succNode.state == frontierSet_node[0]):
                        StateInFrontierSet = True
                        if (succNode.f < frontierSet_node[1]):
                            ExistsBetterPriority = True
                            frontierSet.remove(frontierSet_node)
                            #print 'Remove ',repr((frontierSet_node[0], frontierSet_node[1]))

                            #Recurso'i:
                            for prio, count, frontierNode in frontier.heap:
                                if frontierNode.state == succNode.state:
                                    frontier.heap.remove((prio, count, frontierNode))
                            """
                            Recurso'i. Hay que cambiar la estructura de los nodos para que contenga solo el action_cost, en lugar del path_cost
                            y para guardar la solucion tener una estructura aparte a la que se le van appendeando las acciones,
                            o capaz seguir la implementacion del libro y hacer una funcion con el nodo como parametro y calcula la solucion,
                            o hacer que frontier solo tenga los estados?
                            frontier.update(succNode, succNode.path_cost) con esta operacion deberia de bastar
                            """
                            break

                if not (StateInFrontierSet and not ExistsBetterPriority): # El caso en que no se hace nada es cuando ya esta en la frontera
                # pero con una mejor o igual prioridad
                    frontier.push(succNode, succNode.f)
                    #print 'Push ',repr((succNode.state, succNode.f))
                    frontierSet.add((succNode.state, succNode.f))


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
