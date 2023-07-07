from rclpy import logging
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.task import Future
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup

class CustomActionClient:

    def __init__(self, node, feedback_callback, result_callback, ros_action_name):
        self.action_name = ''
        self.logger = logging.get_logger(self.__class__.__name__) #self.__class__.__name__ otherwise __class__.__name__ print CustomActionClient
        self.callback_group = ReentrantCallbackGroup()
        self.__action_client = ActionClient(node, NavigateToPose, ros_action_name, callback_group=self.callback_group)#ARG
        self._action = None
        self._params = []
        self._goal_handle = None
        self.action_done_event = Event()
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback
        self.future_handle = None
        
    def send_action_goal(self, actionInstance, params, future):
        self.logger.info(f"Starting action '{self.action_name}'")
        self._action = actionInstance
        self._params = params
        self.future_handle = future
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        
        goal_msg=self.create_goalmsg(goal_msg)
    
        self._send_goal_future = self.__action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def create_goalmsg(self, goal_msg):
        return goal_msg

        
    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.info('Error! Goal rejected')
            return
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        self.result_callback(self._action, self._params, future.result().status)
        status = future.result().status
        # TODO: replace magic number -> 4 is the code for successfully completed action in NavigateToPose action format, 5 for canceled action
        if status == 4:
            self.future_handle.set_result("Finished")

    def cancel_action_goal(self):
        self.logger.info('Cancelling goal')
        try:
            self._goal_handle.cancel_goal()
        except:
            self.logger.info('Error! No valid goal handle')