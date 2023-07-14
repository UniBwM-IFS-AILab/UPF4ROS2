from upf4ros2_demo.action_clients.customaction_client  import CustomActionClient



class TakeOffActionClient(CustomActionClient):
    """
    <summary>
    
    :param Node node: <description>
    :param Callable feedback_callback: <description>
    :param Callable result_callback: <description>
    :param string drone_prefix: <description>
    """
    def __init__(self, node, feedback_callback, result_callback, drone_prefix):
        action_name = drone_prefix + "takeoff"
        super().__init__(node, feedback_callback, result_callback, action_name)
        self.action_name="Take Off"
