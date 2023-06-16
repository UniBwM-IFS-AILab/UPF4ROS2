from rclpy import logging

from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# TODO: remove duplicate code in action clients
class TakeOffActionClient():
    def __init__(self, node, feedback_callback, result_callback):
        self.logger = logging.get_logger(__class__.__name__)

        self.__takeoff_client = ActionClient(node, NavigateToPose, '/takeoff')

        self._action = None
        self._params = []
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback

    def send_action_goal(self, actionInstance, params):
        # not working in current state -> blocks without publishing message
        #while not self.__takeoff_client.wait_for_server():
        #    self.logger.info("'Takeoff' action server not available, waiting...")
        self.logger.info("Starting action 'Take Off'")
        self._action = actionInstance
        self._params = params

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = "map"
        
        self._send_goal_future = self.__takeoff_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.logger.info("End 'Take Off'")
    
    def cancel_goal(self):
        #self.__takeoff_client.cancel_goal(self._send_goal_future)
        None

    def goal_response_callback(self, future):
        self.logger.info("goal_response_callback 'Take Off'")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.logger.info('Error! Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        self.logger.info("End 'Take Off'")

    def get_result_callback(self, future):
        self.logger.info("get_result_callback 'Take Off'")
        self.result_callback(self._action, self._params, future.result().result)
        self.logger.info("End 'Take Off'")

    def cancel_action_goal(self):
        None
