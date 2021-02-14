# PacMan With Search Algorithms

In this project Search algorithms and related functions have been developed in line with shared codes [1]. The project was explained with this report. Pacman agent food in line with the project depth first search (DFS), breadth first search (BFS), uniform cost search (UCS) and A * search algorithms are used.
All of the auxiliary codes for the project are written in Python 2.7. New in Python 3 Python 3 system in the coding phase, as the code structure was changed with the regulations Sub-version 2.7 is used instead.

The project is used to run using the general pattern given in gameState.py on the terminal. Of used mold its general definition is as in the commands.txt file.
 
The following search algorithms have been implemented and the scenarios specified has been applied. The implemented algorithms are as follows;
I.DFS
II. BFS
III. UCS
IV. A *

<b>The Python files mentioned in terms of content and their explanations are as follows.</b><br>
pacman.py - Pacman is the main file where the game is executed. Allows creation of Pacman for GameState type.
game.py - Pacman forms the logic part of the game, ie the basis.
util.py - Contains the data structures required for search algorithms.
graphicsDisplay.py - Provides game graphics.
graphicsUtils.py - Allows rendering of graphics.
textDisplay.py - Provides ASCII graphics for Pacman.
ghostAgents.py - Includes agents that control ghosts.
keyboardAgents.py - Provides keyboard controls.
layout.py - allows to read layout files and store their contents.

<b>The contents of search.py and searchAgents.py used for the search are as follows.</b><br>
search.py- There are four different algorithms in the Search Problem class (depthFirstSearch,breadthFirstSearch,uniformCostSearch,aStarSearch)
searchAgents.py- It includes the goal and instinctual functions of the Pacman agent. (SearchAgent, CornersProblem, cornersHeuristic, AStarCornersAgent,FoodSearchProblem, foodHeuristic,ClosestDotSearchAgent,AnyFoodSearchProblem)
