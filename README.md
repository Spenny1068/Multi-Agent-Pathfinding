# Visual demonstration of solving Multi-Agent-Pathfinding (MAPF) problems
![](1.gif)
### What is happening
* Given several independent agents (in this case 7), the program will solve multi-agent pathfinding (MAPF) problem, and animate the solution path for each agent.
* Each agent is travelling along the optimal path (shortest path & shortest running time theoretically possible) from its start to destination.
* *Inspiration**: Amazon warehouse AI robots: https://www.youtube.com/watch?v=HSA5Bq-1fU4  @ 33 seconds in

### What is the Multi-Agent-Pathfinding (MAPF) problem?
* MAPF is an optimization problem in the field of AI.
* It involves finding the optimal collision-free path for multiple agents in an environment while considering their individual goals and constraints
* MAPF is search problem, typically solved using algorithms like A* search, Dijkstra's algorithm, or variants specifically designed for multi-agent scenarios.
* MAPF is a computationally challenging problem, especially when the environment is complex, and there are many agents with conflicting goals and constraints.

### How the program works
* 2-dimensional A-Star Search is used under the hood to find an optimal route from start to destination for each independant agent, considering both spacial obstacles and other agents acting as obstacles at a particular timestep. 
* Dijkstra's Algorithm is used to pre-compute the heuristic function for each cell in the map, which is then used by the A-Star algorithm to find the optimal route.
* Conflict Based Search (CBS) is used find a collision-free solution for all agents, such that each agent is travelling along its optimaml path.

### Different MAPF algorithms used in the program 
#### Independent algorithm
* Plans for all the agents independently. Their paths do not collide with the environment, but are allowed to collide with the paths of other agents. Therefore, collisions are allowed
#### Prioritized algorithm
* Orders the agents by assigning each agent a different priority. It then plans paths for the agents, one after the other, in order of decreasing priority. 
* Prioritized planning is fast but suboptimal (meaning that it does not always find an optimal collision-free solution) and incomplete (meaning that it does not always find a collision-free solution even if one exists) 
#### CBS algorithm
* Plans the shortest paths for all the agents independently. If theres a collision between agents, it recursively considers two cases:
1. the (negative) constraint that prohibits agent a from being in cell x at time step t 
2. the (negative) constraint that prohibits agent b from being in cell x at time step t. 
* Eventually, CBS finds a collision-free solution that is complete and optimal

### Running the demo
Run the program like so:

```python
python3 run_experiments.py --instance instances/<test_case.txt> --solver <algorithm>
```

E.g, The above animation can be run by using
```python
python3 run_experiments.py --instance instances/test_21.txt --solver CBS
```
