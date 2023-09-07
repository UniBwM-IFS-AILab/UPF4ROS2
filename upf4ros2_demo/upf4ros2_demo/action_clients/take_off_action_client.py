from upf4ros2_demo.action_clients.custom_action_client import CustomActionClient

from as2_msgs.action import TakeOff

class TakeOffActionClient(CustomActionClient):
    """
    Action client for the take_off action
    
    :param Node node: the plan executor node
    :param Callable feedback_callback: function pointer for feedback callback function in plan executor
    :param Callable result_callback: function pointer for result callback function in plan executor
    :param string drone_prefix: drone identifier for the action client, so the correct respective server is called (for multiple drones)
    """
    def __init__(self, node, feedback_callback, result_callback, drone_prefix):
        action_name = drone_prefix + "takeoff"
        super().__init__(node, feedback_callback, result_callback, action_name, TakeOff)
        self.action_name="Take Off"

    def create_goalmsg(self, goal_msg):
        """
        :param self._action_type goal_msg: the goal message to be passed to the action server
        
        :returns: goal_msg: The goal message that is passed to the action server
        :rtype: self._action_type msg
        """
        goal_msg.takeoff_height = 10
        return goal_msg