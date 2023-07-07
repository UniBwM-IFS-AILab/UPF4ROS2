from upf4ros2_demo.action_clients.customaction_client import CustomActionClient


class LandActionClient(CustomActionClient):

    def __init__(self, node, feedback_callback, result_callback, drone_prefix):
        action_name = drone_prefix + "landing"
        super().__init__(node, feedback_callback, result_callback, action_name)
        self.action_name="Land"

