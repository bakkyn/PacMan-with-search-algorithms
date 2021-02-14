import util


class SearchProblem:
    """
    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        fileName, line, method
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
        (successor, action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
        actions: A list of actions to take
        This method returns the total cost of a particular sequence of actions.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(dfs):
    # preparations for algorithm
    startPoint = dfs.getStartState()  # returns the start state for the search problem.
    stack = util.Stack()  # stack to push pop operations
    visited = []  # is it visited

    node = (startPoint, [])  # state, action

    stack.push(node)  # First node

    while not stack.isEmpty():

        nextNode = stack.pop()  # len - 1 = top
        state = nextNode[0]
        acts = nextNode[1]

        if dfs.isGoalState(state):  # if node is the goal, break
            return acts

        if state not in visited:  # it does not visit before
            visited.append(state)
            succ = dfs.getSuccessors(state)  # get possible succ = successor, action, stepCost

            for succState, succAction, succCost in succ:
                newAction = acts + [succAction]
                newNode = (succState, newAction)
                stack.push(newNode)

    return acts


def breadthFirstSearch(bfs):
    # Preparations for algorithm
    startPoint = bfs.getStartState()  # Returns the start state for the search problem.
    queue = util.Queue()  # queue to apply operations
    visited = []  # is it visited

    node = (startPoint, [])  # state, action, cost

    queue.push(node)  # First node

    while not queue.isEmpty():

        nextNode = queue.pop()  # len - 1 = top
        state = nextNode[0]
        acts = nextNode[1]

        if bfs.isGoalState(state):  # if node is the goal, break
            return acts

        if state not in visited:  # it does not visit before
            visited.append(state)
            succ = bfs.getSuccessors(state)  # get possible succ = successor, action, stepCost

            for succState, succAction, succCost in succ:
                newAction = acts + [succAction]
                newNode = (succState, newAction)
                queue.push(newNode)

    return acts


def uniformCostSearch(ucs):
    # Preparations for algorithm
    startPoint = ucs.getStartState()  # Returns the start state for the search problem.
    priorityQ = util.PriorityQueue()  # PriorityQueue to apply operations
    visited = []

    node = (startPoint, [])  # state, action

    priorityQ.push(node, 0)  # First node ( node , cost )

    while not priorityQ.isEmpty():

        nextNode = priorityQ.pop()  # len - 1 = top
        state = nextNode[0]
        acts = nextNode[1]

        if ucs.isGoalState(state):  # if node is the goal, break
            return acts

        if (state not in visited):  # it does not visit before
            visited.append(state)

            succ = ucs.getSuccessors(state)  # get possible succ = successor, action, stepCost

            for succState, succAction, succCost in succ:
                newAction = acts + [succAction]

                priorityQ.push((succState, newAction), ucs.getCostOfActions(newAction))

    return acts


def nullHeuristic(state, problem=None):
    return 0


def aStarSearch(aStr, heuristic=nullHeuristic):

    startPoint = aStr.getStartState()  # returns the start state for the search problem.
    priorityQ = util.PriorityQueue()    # PriorityQueue to apply operations
    visited = []

    node = (startPoint, [])  # state, action

    priorityQ.push(node, 0)  # First node - node , cost
    while not priorityQ.isEmpty():

        nextNode = priorityQ.pop()  # len - 1 = top
        state = nextNode[0]
        acts = nextNode[1]

        if (aStr.isGoalState(state)):
            break

        if (state not in visited):  # it does not visit before or cost is lower than others
            visited.append(state)

            succ = aStr.getSuccessors(state)  # get possible succ = successor, action, stepCost

            for succState, succAction, succCost in succ:
                newAction = acts + [succAction]
                newCostH = aStr.getCostOfActions(newAction) + heuristic(succState, aStr)

                priorityQ.push((succState, newAction), newCostH)

    return acts

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
