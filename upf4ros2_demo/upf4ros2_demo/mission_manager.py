import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from upf4ros2_demo.pddl_from_sg import ProblemMSG
from gtsg.monitorGame import MonitorSG
import numpy as np
import os

from upf4ros2_demo_msgs.action import Mission
#from upf4ros2_demo_msgs.srv import CallMission

from std_msgs.msg import String
import time



class MissionManager(Node):
    """
    Class use to handle the generation of problem and launch request to the
    plan executor node as well as launching new request when
    the previous are done
    """

    def __init__(self,model):
        super().__init__('game_manager')
        self.nb_drone=2
        # Model: Stochastic game or another custom model (MDP,...)
        self.model=model
        # Problem: Object to convert the current model into a set of PDDL problem
        self.problem=self.problem_model()
        self.nb_result_received=0
        self.action_clients=[]
        for i in range(self.nb_drone):
            self.action_clients+=[ActionClient(self, Mission, f'mission{i}')]
        

    def problem_model(self):
        """
        Return the class used to create the PDDL files from the model used
        :return:
        :rtype: ProblemMSG
        """
        return ProblemMSG(self.model,5)
    

    #Gam solver with status argument
    def solve_problem_model_status(self,status):
        """
        For a given status, generate the problem file of the
        corresponding solution
        :param status: State of the game to solve
        """
        self.get_logger().info("Start gen pddl")
        self.problem.problem_gen(status,0)
        self.get_logger().info("End gen pddl")
              
        
    #Action launcher with status as arg
    def launch_mission(self,init_status):
        """
        Launch one mission cycle (creating problem and requesting an execution)
        :param init_status: Initial state at the beginning of the cycle
        """
        self.get_logger().info("Begin Mission")
        goal_msg=Mission.Goal()
        goal_msg.init_status=init_status
        # Create the problem file for the current state
        self.problem.problem_gen(tuple(init_status),0)
        self.get_logger().info("Gen end")
        # Wait  for the plan_executor node (drone) to start
        for i in range(self.nb_drone):
            self.action_clients[i].wait_for_server()
        self.get_logger().info("Wait End")
        li_send_goal_future=[]
        time.sleep(2)
        # Send an action request to each node to compute and execute
        # the problem plan
        for i in range(self.nb_drone):
            tmp_send_goal_future=self.action_clients[i].send_goal_async(goal_msg)
            tmp_send_goal_future.add_done_callback(self.goal_response_callback)
            li_send_goal_future+=[tmp_send_goal_future]
            time.sleep(2)
        

    def goal_response_callback(self, future):
        """
        Callback function for the action
        :param future:
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
        
    def get_result_callback(self, future):
        """
        Result callback function for the mission action, launching another
        action request when every drone finish their previous one
        :param future:
        """
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_status))
        self.nb_result_received+=1
        # Wait for every execution node to send the result back
        # (meaning the mission ended succesfully )
        if self.nb_result_received==self.nb_drone:
            self.nb_result_received=0
            #Implmentation of update of state in there if necessary
            new_state=result.final_status
            self.launch_mission(new_state)
    


def main(args=None):
    # Distance Matrix between drone and site, set to 0
    tmp_zeros = np.zeros((2, 4))
    # Hardcoded Monitor game
    mobis = MonitorSG(2, 4,
                      [[0, 0, 0, 0], [1, 1, 1, 1]],
                      [
                       [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
                       [[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
                       [[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
                       [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]],
                      tmp_zeros
                      )
    rclpy.init()
    gamenode=MissionManager(mobis)
    # Hardcoded state initial status
    status=[0,1,1,1]
    gamenode.launch_mission(status)
    rclpy.spin(gamenode)

    gamenode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()


# [[[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
# [[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
# [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
# [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]]

# [[[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
# [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
# [[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
# [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]],