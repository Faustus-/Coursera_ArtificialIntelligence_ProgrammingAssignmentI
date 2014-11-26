# search.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html
import pacman
from pacman import GameState

"""
In search.py, you will implement generic search algorithms which are called 
by Pacman agents (in searchAgents.py).
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
  """
  Search the deepest nodes in the search tree first [p 74].
  
  Your search algorithm needs to return a list of actions that reaches
  the goal.  Make sure to implement a graph search algorithm [Fig. 3.18].
  
  To get started, you might want to try some of these simple commands to
  understand the search problem that is being passed in:
  
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  """
  "*** YOUR CODE HERE ***"
  '''
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  '''
  # initialize a stack
  frontier = util.Stack()
  # initialize a set to mark the states visited
  visited = util.Counter() 
  # push current state into stack
  # frontier structure is (state, route to this state)
  frontier.push((problem.getStartState(),[]))
  # deep first search
  while not(frontier.isEmpty()):
      # pop a state
      currentState = frontier.pop();
      # mark the state
      if visited[currentState[0]] == 0:
          visited[currentState[0]] += 1
          # check the goal
          if problem.isGoalState(currentState[0]):
              # return the route
              return currentState[1]
          for child in problem.getSuccessors(currentState[0]):
              # push all approachable states into fringe
              ''' there is a bug when two or more same unvisited states in the stack
              if visited[child[0]] == 0:
              frontier.push((child[0], currentState[1]+ [child[1], ]))
              '''
              frontier.push((child[0], currentState[1]+ [child[1], ]))
  '''
  util.raiseNotDefined()
  '''
    
  

def breadthFirstSearch(problem):
  "Search the shallowest nodes in the search tree first. [p 74]"
  "*** YOUR CODE HERE ***"
  
  # initialize a queue
  frontier = util.Queue()
  # initialize a set to mark the states visited
  visited = util.Counter() 
  # push current state into queue
  # frontier structure is (state, route to this state)
  frontier.push((problem.getStartState(),[]))
  # deep first search
  while not(frontier.isEmpty()):
      # pop a state
      currentState = frontier.pop();
      # mark the state
      if visited[currentState[0]] == 0:
          visited[currentState[0]] += 1
          # check the goal
          if problem.isGoalState(currentState[0]):
              # return the route
              # print currentState[1]
              return currentState[1]
          for child in problem.getSuccessors(currentState[0]):
              # push all approachable states into fringe
              ''' there is a bug when two or more same unvisited states in the stack
              if visited[child[0]] == 0:
              frontier.push((child[0], currentState[1]+ [child[1], ]))
              '''
              if visited[child[0]] == 0:
                frontier.push((child[0], currentState[1]+ [child[1], ]))
  #util.raiseNotDefined()
      
def uniformCostSearch(problem):
  "Search the node of least total cost first. "
  "*** YOUR CODE HERE ***"
  '''
  print "Start:", problem.getStartState()
  print "Is the start a goal?", problem.isGoalState(problem.getStartState())
  print "Start's successors:", problem.getSuccessors(problem.getStartState())
  '''
  # initialize a priority queue
  frontier = util.PriorityQueue()
  # initialize a set to mark the states visited
  visited = util.Counter() 
  # push current state into queue
  # frontier structure is (state, route to this state, priority of this state)
  frontier.push((problem.getStartState(),[],0), 0)
  # deep first search
  while not(frontier.isEmpty()):
      # pop a state
      currentState = frontier.pop();
      # mark the state
      if visited[currentState[0]] == 0:
          visited[currentState[0]] += 1
          # check the goal
          if problem.isGoalState(currentState[0]):
              # return the route
              return currentState[1]
          for child in problem.getSuccessors(currentState[0]):
              # push all approachable states into fringe
              ''' there is a bug when two or more same unvisited states in the stack
              if visited[child[0]] == 0:
              frontier.push((child[0], currentState[1]+ [child[1], ]))
              '''
              frontier.push((child[0], currentState[1] + [child[1], ], currentState[2] + child[2]), currentState[2] + child[2])
  # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

def aStarSearch(problem, heuristic=nullHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  
    # initialize a priority queue
  frontier = util.PriorityQueue()
  # initialize a set to mark the states visited
  visited = util.Counter() 
  # push current state into queue
  # frontier structure is (state, route to this state, priority of this state)
  frontier.push((problem.getStartState(),[],0), 0)
  # deep first search
  while not(frontier.isEmpty()):
      # pop a state
      currentState = frontier.pop();
      # mark the state
      if visited[currentState[0]] == 0:
          visited[currentState[0]] += 1
          # check the goal
          if problem.isGoalState(currentState[0]):
              # return the route
              return currentState[1]
          for child in problem.getSuccessors(currentState[0]):
              # push all approachable states into fringe
              ''' there is a bug when two or more same unvisited states in the stack
              if visited[child[0]] == 0:
              frontier.push((child[0], currentState[1]+ [child[1], ]))
              '''
              frontier.push((child[0], currentState[1] + [child[1], ], currentState[2] + child[2]), currentState[2] + child[2] + heuristic(child[0], problem))
  '''
  util.raiseNotDefined()
  '''
              
  
# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
