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
    def __init__(self, node, feedback_callback, result_callback, ros_action_name, action_type = NavigateToPose):
        """
        Constructor method
        
        :param Node node: the plan executor node
        :param Callable feedback_callback: function pointer for feedback callback function in plan executor
        :param Callable result_callback: function pointer for result callback function in plan executor
        :param string ros_action_name: action name for the action client -> has to match with name on server side
        :param .action action_type: type of the msg transmitted to the action server --> has to match with type on server side, must be imported here and on the server side
        """
        self.action_name = ''
        self.node = node
        self.logger = logging.get_logger(self.__class__.__name__)
        self.callback_group = ReentrantCallbackGroup()
        self.__action_client = ActionClient(node, action_type, ros_action_name, callback_group=self.callback_group)#ARG
        self._action_type = action_type
        self._action = None
        self._params = []
        self._goal_handle = None
        self.action_done_event = Event()
        self.feedback_callback = feedback_callback
        self.result_callback = result_callback
        self.future_handle = None
        self.timer = None
        
    def send_action_goal(self, actionInstance, params, future):
        """
        Send an action goal to the action server. The implemented retransmission could also be used with a Feedback mechanism where it checks if the current action really gets executed.
        However this could also be done within offboard_control to scope the implementation
        
        :param actionInstance: executed action
        :param params: params of the action
        :param Future future: future handle that will be set when the action is finished -> plan executor gets notified and starts next action
        """
        
        while not self.__action_client.wait_for_server(2):
            self.logger.info("Action server not available, waiting...")
        
        self.logger.info(f"Starting action '{self.action_name}'")
        self._action = actionInstance
        self._params = params
        self.future_handle = future
        goal_msg = self._action_type.Goal()
        
        goal_msg = self.create_goalmsg(goal_msg)
        self.send_goal_msg(goal_msg)
        
    def send_goal_msg(self, goal_msg):
        self._send_goal_future = self.__action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        # self.retransmit_goal_after_timeout(5, goal_msg)
        # the timer should be canceled when progressing to goal_response_callback
        # do this via adding an additional add_done_callback
        #self._send_goal_future.add_done_callback(lambda f: self.timer.cancel())
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
    def retransmit_goal_after_timeout(self, timeout, goal_msg):
        # Cancel the previous timer so it doesn't run again
        if self.timer:
            self.timer.cancel()
        def delayed_retransmission():
            self.logger.info('Retransmission!')
            
            # retransmit goal_msg -> status is not accepted yet because otherwise the timer would have been canceled already by the add_done_callback
            # cancel previous future
            if self._send_goal_future and not self._send_goal_future.done():
                self.logger.info('canceling previous future!')
                self._send_goal_future.cancel()
                self.logger.info('after canceling previous future!')
                self.send_goal_msg(goal_msg)
                
        self.timer = self.node.create_timer(timeout, delayed_retransmission)
    
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
        
        :param self._action_type goal_msg: the goal message to be passed to the action server
        
        :returns: goal_msg: The goal message that is passed to the action server
        :rtype: self._action_type msg
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
        self.logger.info('Goal accepted by action server')
        self._get_result_future = self._goal_handle.get_result_async()
        #self.timer = self.node.create_timer(5, lambda : self.logger.info('result future not done yet'))
        
        self._get_result_future.add_done_callback(self.get_result_callback)
        #self._get_result_future.add_done_callback(lambda f: self.timer.cancel())

        
        
    def get_result_callback(self, future):
        """
        Handles the result from the action server and forwards it to the plan executor
        
        :param Future future: the future containing the response from the action server
        
        """
        self.result_callback(self._action, self._params, future.result())
        status = future.result().status
        # self.logger.info(f"result status: {status}")
        # GoalStatus.STATUS_SUCCEEDED -> 4 is the status for successfully completed action in the GoalStatus Message, 5 for canceled action (GoalStatus.STATUS_CANCELED)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.future_handle.set_result("Finished")

    def feedback_callback(self, feedback_msg):
        """
        Processes Feedback messages from the action server and forwards it to the plan executor, can be overwritten in inheriting classes
        
        :param _action_type.Feedback() feedback_msg: the message containing the feedback from the action server
        
        """
        feedback = feedback_msg.feedback
        # feedback.<attribute>
        # TODO forward to plan_executor