import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        # constraint for 1.2
        # { 'agent': 0, 'loc': [(1, 5)], 'timestep': 4 }  
        # constraint for 1.3
        # { 'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1 }
        # constraint for 1.4
        # { 'agent': 0, 'loc': [(1, 5)], 'timestep': 10 }
        # constraints for 1.5
        # { 'agent': 1, 'loc': [(1, 4)], 'timestep': 2 }
        # { 'agent': 1, 'loc': [(1, 3)], 'timestep': 2 }
        # { 'agent': 1, 'loc': [(1, 2)], 'timestep': 2 }


        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            # push entire solution path of current agent into constraints list for future agents to consider
            # this handles task 2.3 as well because it adds vertex constraints at all timesteps the agent is sitting at its goal state
            for ts in range(len(path)):

                # push vertex constraints
                vertexConstraint_loc = [None] * 1
                vertexConstraint_loc[0] = path[ts]
                vertexConstraint = {'agent': i + 1, 'loc': vertexConstraint_loc, 'timestep': ts}
                constraints.append(vertexConstraint)

                # push edge constraints

                # don't push edge collisions of the form [(1, 4), (1, 4)] 
                if path[ts - 1] == path[ts]:
                    continue

                edgeConstraint_loc = [None] * 2
                edgeConstraint_loc[0] = path[ts - 1]
                edgeConstraint_loc[1] = path[ts]
                edgeConstraint = {'agent': i + 1, 'loc': edgeConstraint_loc, 'timestep': ts}
                constraints.append(edgeConstraint)

            print("constraints for agent ", str(i + 1), " = ", str(constraints))

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
