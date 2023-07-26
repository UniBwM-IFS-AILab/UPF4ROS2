from rclpy import logging
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from rclpy.task import Future
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup

class CustomActionClient:
    """
    Template class used to create an action client
    
    
    """
    def __init__(self, node, feedback_callback, result_callback, ros_action_name):
        """
        Constructor method
        
        :param Node node: the plan executor node
        :param Callable feedback_callback: function pointer for feedback callback function in plan executor
        :param Callable result_callback: function pointer for result callback function in plan executor
        :param string ros_action_name: action name for the action client -> has to match with name on server side
        """
        self.action_name = ''
        self.logger = logging.get_logger(self.__class__.__name__)
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
        """
        Send an action goal to the action server
        
        :param actionInstance: executed action
        :param params: params of the action
        :param Future future: future handle that will be set when the action is finished -> plan executor gets notified and starts next action
        """
        self.logger.info(f"Starting action '{self.action_name}'")
        self._action = actionInstance
        self._params = params
        self.future_handle = future
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        
        goal_msg = self.create_goalmsg(goal_msg)
    
        self._send_goal_future = self.__action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def cancel_action_goal(self):
        """
        Cancels the current action
        
        """
        self.logger.info('Cancelling goal')
        try:
            self._goal_handle.cancel_goal()
        except:
            self.logger.info('Error! No valid goal handle')


    def create_goalmsg(self, goal_msg):
        """
        Function that can be overwritten by child clients to create custom goal messages
        
        :param NavigateToPose goal_msg: the goal message to be passed to the action server
        
        :returns: goal_msg: The goal message that is passed to the action server
        :rtype: NavigateToPose msg
        """
        return goal_msg

        
    def goal_response_callback(self, future):
        """
        Handles the goal response from the action server
        
        :param Future future: the future containing the response from the action server
        
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.info('Error! Goal rejected')
            return
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """
        Handles the result from the action server and forwards it to the plan executor
        
        :param Future future: the future containing the response from the action server
        
        """
        self.result_callback(self._action, self._params, future.result())
        status = future.result().status
        # GoalStatus.STATUS_SUCCEEDED -> 4 is the status for successfully completed action in the GoalStatus Message, 5 for canceled action (GoalStatus.STATUS_CANCELED)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.future_handle.set_result("Finished")
