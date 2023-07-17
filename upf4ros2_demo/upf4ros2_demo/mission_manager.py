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



class MissionManager(Node):

    def __init__(self,model):
        super().__init__('game_manager')
        self.model=model
        self.problem=self.problem_model()
        self.action_client_mission=ActionClient(self, Mission, 'mission')
        

    def problem_model(self):
        return ProblemMSG(self.model,5)

    #Game solver: default
    def solve_problem_model(self):
        self.get_logger().info("Start")
        self.problem.problem_gen((1,1,1,1),0)
        self.get_logger().info("End")
    
    
    #Gam solver with status argument
    def solve_problem_model_status(self,status):
        self.get_logger().info("Start")
        self.problem.problem_gen(status,0)
        self.get_logger().info("End")
        
        
    # #Callback for subscription
    # def launch_newgame(self,msg):
        # self.problem.problem_gen((0,1,0,0),0)
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # msgbis=String()
        # msgbis.data="GameFinishReload"
        # self.newgame_pub.publish(msgbis)
        
        
    #Action launcher with status as arg
    def launch_mission(self,init_status):
        self.get_logger().info("Begin Mission")
        goal_msg=Mission.Goal()
        goal_msg.init_status=init_status
    
        self.problem.problem_gen(tuple(init_status),0)
        self.get_logger().info("Gen end")
        self.action_client_mission.wait_for_server()
        self.get_logger().info("Wait End")
        self._send_goal_future = self.action_client_mission.send_goal_async(goal_msg)
        self.get_logger().info(f"send goal future")
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f"add_done_callback")
        

    #future callback for action
    def goal_response_callback(self, future):
        self.get_logger().info(f"begin goal_response_callback")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self.get_logger().info(f"add get_result_callback")
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.get_logger().info(f"end goal_response_callback")
        
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_status))
        self.launch_mission(result.final_status)
        #self.current_status=result.final_status
        #return result.final_status
        #rclpy.shutdown()
    
    
    # # Service launcher
    # def send_request(self,status):
        # self.req_cus.init_status=list(status)
        # self.solve_problem_model_status(status)
        # self.get_logger().info("SolveGameEnd")
        # self.future = self.client_mission.call_async(self.req_cus)


def main(args=None):
    tmp_zeros = np.zeros((2, 4))
    mobis = MonitorSG(2, 4,
                      [[0, 0, 0, 0], [1, 1, 1, 1]],
                      [[[1 - 0.855, 0.855], [1 - 0.64, 0.64]],
                       [[1 - 0.64, 0.64], [1 - 0.35, 0.35]],
                       [[1 - 0.56, 0.56], [1 - 0.36, 0.36]],
                       [[1 - 0.42, 0.42], [1 - 0.42, 0.42]]],
                      tmp_zeros
                      )
    rclpy.init()
    gamenode=MissionManager(mobis)
    gamenode.solve_problem_model()
    status=[0,0,0,1]
    #gamenode.launch_mission(status)
    rclpy.spin(gamenode)

    gamenode.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
