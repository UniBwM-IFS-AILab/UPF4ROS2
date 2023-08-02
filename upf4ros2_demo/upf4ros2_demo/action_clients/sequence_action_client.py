import json
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from upf4ros2_demo.action_clients.custom_action_client import CustomActionClient



class SequenceActionClient(CustomActionClient):
    """
    Action client for sequences of multiple actions. 
    It can be used if the connection to ground control is not guaranteed to be stable and the drone should continue the plan without having to send every result back or having to receive new commands.
    It should use another type of action to contain multiple waypoints: NavigateThroughPoses.action
    
    :param Node node: the plan executor node
    :param Callable feedback_callback: function pointer for feedback callback function in plan executor
    :param Callable result_callback: function pointer for result callback function in plan executor
    :param string drone_prefix: drone identifier for the action client, so the correct respective server is called (for multiple drones)
    """
    def __init__(self, node, feedback_callback, result_callback, drone_prefix):
        action_name = drone_prefix + "action_sequence"
        super().__init__(node, feedback_callback, result_callback, action_name, NavigateThroughPoses)
        lookupTablePath = (get_package_share_directory('upf4ros2_demo')
                                + str('/params/lookupTable.json'))
        self._lookupTable = dict()
        with open(lookupTablePath) as file:
            self._lookupTable = json.load(file)
        # set initial home coordinates; overwrite later
        # self._lookupTable['home'] = [0,0,0]
        self.action_name="ActionSequence"
    
    
    def create_goalmsg(self, goal_msg):
        # self._action should in this case contain an array of actions
        # iterate over this array and construct a NavigateThroughPoses message
        # put a comma separated string sequence (TODO: make it xml instead) of the actions into: string behavior_tree
        
        for i in self._action:
            actionName = i.action_name
            goal_msg.behavior_tree += (actionName + ",")
            
            if actionName == "take_off" or actionName == "land":
                # specific logic here if required
                pass
            elif actionName == "fly":
                wp = i.parameters[2].symbol_atom[0]
                wp_resolved = self._lookupTable[wp]
                wp_pose = PoseStamped()
                wp_pose.pose.position.x = float(wp_resolved[0])
                wp_pose.pose.position.y = float(wp_resolved[1])
                wp_pose.pose.position.z = float(wp_resolved[2])
                wp_pose.header.frame_id = "map"
                goal_msg.poses.append(wp_pose)
        
        # remove trailing comma with your_string.rstrip(',')
        goal_msg.behavior_tree.rstrip(',')
        
        return goal_msg
