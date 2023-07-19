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

    def __init__(self,model):
        super().__init__('game_manager')
        self.nb_drone=2
        self.model=model
        self.problem=self.problem_model()
        self.nb_result_received=0
        self.action_clients=[]
        for i in range(self.nb_drone):
            self.action_clients+=[ActionClient(self, Mission, f'mission{i}')]
        

    def problem_model(self):
        return ProblemMSG(self.model,5)
    

    #Gam solver with status argument
    def solve_problem_model_status(self,status):
        self.get_logger().info("Start gen pddl")
        self.problem.problem_gen(status,0)
        self.get_logger().info("End gen pddl")
              
        
    #Action launcher with status as arg
    def launch_mission(self,init_status):
        self.get_logger().info("Begin Mission")
        goal_msg=Mission.Goal()
        goal_msg.init_status=init_status
        self.problem.problem_gen(tuple(init_status),0)
        self.get_logger().info("Gen end")
        for i in range(self.nb_drone):
            self.action_clients[i].wait_for_server()
        self.get_logger().info("Wait End")
        li_send_goal_future=[]
        time.sleep(2)
        for i in range(self.nb_drone):
            tmp_send_goal_future=self.action_clients[i].send_goal_async(goal_msg)
            tmp_send_goal_future.add_done_callback(self.goal_response_callback)
            li_send_goal_future+=[tmp_send_goal_future]
            time.sleep(2)
        

    #future callback for action
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_status))
        self.nb_result_received+=1
        if self.nb_result_received==self.nb_drone:
            self.nb_result_received=0
            self.launch_mission(result.final_status)
        #self.current_status=result.final_status
        #return result.final_status
        #rclpy.shutdown()
    


def main(args=None):
    tmp_zeros = np.zeros((2, 4))
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
    #gamenode.solve_problem_model()
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