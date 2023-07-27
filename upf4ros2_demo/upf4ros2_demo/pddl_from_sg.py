from unified_planning.shortcuts import *
from unified_planning.io import PDDLReader, PDDLWriter
import itertools
from ament_index_python.packages import get_package_share_directory
import numpy as np

class ProblemMSG:
    """
    Class used to compute the policy of a Monitoring game and turn it into a
    pddl problem depending of the state and horizon

    :param MSG monitorsg: The monitoring game to used as a model for the problem
    :param int horizon: Horizon of computation of the policy
    """
    def __init__(self,monitorsg,horizon):
        self.monsg = monitorsg
        self.policy=self.monsg.sg.backwardRecursion(horizon)
        reader = PDDLReader()
        self.domain=reader.parse_problem(get_package_share_directory('upf4ros2_demo')+'/pddl/monitor_domain.pddl')

    def problem_gen(self,state,horizon):
        """
        Method used to generate the pddl problem file for each player for a
        given state at a given horizon

        :param list state: Current state of the stochastic game
        :param int horizon: Current horizon of the stochastic game
        """
        drone_type = self.domain.user_type("drone")
        waypoint_type = self.domain.user_type("waypoint")

        at_pred = self.domain.fluent("at")
        landed_pred = self.domain.fluent("landed")
        takoff_pred = self.domain.fluent("taken_off")
        inspected_pred = self.domain.fluent("inspected")
        state_i=list(itertools.product(
            *[range(2) for m in range(self.monsg.m_sites)])).index(state)
        state_policy=self.policy[horizon][state_i]
        problems=[]
        for n in range(self.monsg.n_players):
            strat_n=state_policy[n]
            # If there is a mixed strategy, a random action is choosen
            played_act=np.random.choice(range(self.monsg.m_sites), 1, p=strat_n)[0]
            problem = self.domain.clone()
            home = problem.add_object(f"home{n}", waypoint_type)
            drone = problem.add_object(f"d{n}", drone_type)
            waypoints = [unified_planning.model.Object(f"w{m}", waypoint_type) for m
                         in range(self.monsg.m_sites)]
            problem.add_objects(waypoints)

            problem.set_initial_value(at_pred(drone, home), True)
            problem.set_initial_value(landed_pred(drone), True)

            problem.add_goal(at_pred(drone, home))
            problem.add_goal(landed_pred(drone))
            problem.add_goal(inspected_pred(waypoints[played_act]))

            problems+=[problem]
            w=PDDLWriter(problem)
            w.write_problem(get_package_share_directory('upf4ros2_demo')+ f"/pddl/monitorproblem_{n}.pddl")
        # For short predicate (at), the problem generator add
        # an _ to the name (at_). We rewrite the domain file for this reason
        w = PDDLWriter(problems[0])
        w.write_domain(get_package_share_directory('upf4ros2_demo') + "/pddl/monitor_domain_upf.pddl")

