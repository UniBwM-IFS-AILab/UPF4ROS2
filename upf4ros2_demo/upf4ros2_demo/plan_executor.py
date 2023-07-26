import rclpy

from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf_msgs import msg as msgs

from upf4ros2_demo.action_clients.take_off_action_client import TakeOffActionClient
from upf4ros2_demo.action_clients.land_action_client import LandActionClient
from upf4ros2_demo.action_clients.fly_action_client import FlyActionClient
from upf4ros2_demo.action_clients.inspect_action_client import InspectActionClient
from upf4ros2_demo.action_clients.custom_action_client import CustomActionClient

from upf_msgs.action import (
    PDDLPlanOneShot
)

from upf_msgs.srv import (
    GetProblem,
    SetInitialValue,
    Replan
)
from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv
from upf4ros2_demo_msgs.action import Mission

class PlanExecutorNode(Node):
    """
    Ros2 node used to create and execute a plan
    
    """

    def __init__(self):
        """
        Constructor method
        """
        super().__init__('plan_executor')
        
        # declare params
        self.declare_parameter('domain', '/pddl/uav_domain.pddl')
        self.declare_parameter('problem', '/pddl/generated_uav_instance.pddl')
        self.declare_parameter('drone_prefix', rclpy.Parameter.Type.STRING)
        # Separate PDDL files for stochastic game
        # self.declare_parameter('domain', '/pddl/monitor_domain_upf.pddl')
        # self.declare_parameter('problem', '/pddl/monitorproblem_0.pddl')

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')
        self._drone_prefix = self.get_parameter('drone_prefix').get_parameter_value().string_value
        
        # the problem name that gets registered in upf4ros2 main
        self._problem_name = self._drone_prefix + 'uav_problem'
        # stores the problem instance in UPF format
        self._problem = None
        # maps all availabe action names to their UPF representation
        self._actions = {}
        # maps all availabe object names to their UPF representation
        self._objects = {}
        # maps all availabe fluent names to their UPF representation
        self._fluents = {}
        # stores the plan as a sequence of UPF actions
        self._plan = []
        # the future of the current executed action
        self._current_action_future = None
        # the instance of the action client that executes the current action
        self._current_action_client = None

        # can be used to translate from UPF representation to ros msg
        self._ros2_interface_writer = ROS2InterfaceWriter()
        
        # can be used to translate from ros msg to UPF representation
        self._ros2_interface_reader = ROS2InterfaceReader()
        
        self.sub_node = rclpy.create_node('sub_node_' + self._drone_prefix[0:-1], use_global_arguments=False)

        # create action clients
        self._take_off_client = TakeOffActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._land_client = LandActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._fly_client = FlyActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._inspect_client = InspectActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        
        # maps the action clients to their respective action names from the pddl domain file
        self._action_client_map = {'take_off': self._take_off_client,
                                   'land': self._land_client,
                                   'fly': self._fly_client
                                   }
        
        self._plan_pddl_one_shot_client = ActionClient(self, PDDLPlanOneShot, 'upf4ros2/action/planOneShotPDDL')
        self.mission_action_server = ActionServer(self,Mission,'mission',self.mission_callback)
        
        # create clients
        self._get_problem = self.sub_node.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        self._set_initial_value = self.sub_node.create_client(SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._plan_pddl_one_shot_client_srv = self.sub_node.create_client(PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')
        
        # create services
        replan_server_name = 'upf4ros2/srv/' + self._drone_prefix + 'replan'
        self._replan = self.sub_node.create_service(Replan, replan_server_name, self.replan)
        
        
        
        
        
    def set_initial_value(self, fluent, object, value_fluent):
        """
        Updates the initial value of a problem in the upf4ros problem manager

        :param fluent: the fluent to be updated (e.g. fluent: drone_at)
        :param object: the object(s) for the fluent (e.g. object: waypoint)
        :param value_fluent: the new value (of type <object>) assigned to the fluent (e.g. forest1)
        """        
        srv = SetInitialValue.Request()
        srv.problem_name = self._problem_name
        srv.expression = self._ros2_interface_writer.convert(fluent(*object))

        item = msgs.ExpressionItem()
        item.atom.append(msgs.Atom())
        item.atom[0].boolean_atom.append(value_fluent)
        item.type = 'up:bool'
        item.kind = msgs.ExpressionItem.CONSTANT
        value = msgs.Expression()
        value.expressions.append(item)
        value.level.append(0)

        srv.value = value

        self._set_initial_value.wait_for_service()
        future = self._set_initial_value.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)

    def update_initial_state(self, action, parameters):
        """
        Updates the problem with all the effects from an executed action

        
        :param upf_msgs/Action action: the executed action with it's associated effects
        :param parameters: the parameters (i.e. objects) of the executed action
        """        
        self.get_logger().info("Updating initial state") 
        # maps identifiers of action parameters to their concrete instance
        # e.g. x : myuav, y : urbanArea, z : home
        paramMap = {action.parameters[i].name : parameters[i]  for i in range(len(parameters))}
        # loop through all effects of the action and update the initial state of the problem accordingly
        for effect in action.effects:
            fluent = effect.fluent.fluent()
            # there seems to be a bug in UPF where fluent.fluent() unpacks the arguments from the predicate declaration instead of  -> workaround: access the arguments directly with effect.fluent.args
            fluent_signature = [self._objects[paramMap[x.parameter().name]] for x in effect.fluent.args]
            value = effect.value.constant_value()
            self.set_initial_value(fluent, fluent_signature, value)


    def get_problem(self):
        """
        Retrieves the current state of the problem from the upf4ros problem manager

        :returns: problem: The current state of the problem
        :rtype: upf_msgs/Problem
        """        
        srv = GetProblem.Request()
        srv.problem_name = self._problem_name

        self._get_problem.wait_for_service()
        future = self._get_problem.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)
        problem = self._ros2_interface_reader.convert(future.result().problem)
        return problem
    
    def get_plan_from_pddl(self):
        """
        This function reads in the pddl domain file and pddl problem file as specified in the init function.
        These files are then passed to upf4ros2 main where a plan is calculated. The resulting plan (along with actions, objects, fluents)
        are then saved in this node, so the plan can be executed.
        

        """        
        self.get_logger().info('Planning...')
        srv = PDDLPlanOneShotSrv.Request()
        srv.plan_request.mode = msgs.PDDLPlanRequest.FILE
        self._problem = self.get_parameter('problem')
        srv.plan_request.domain = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._domain.value))
        srv.plan_request.problem = (get_package_share_directory('upf4ros2_demo')
                                        + str(self._problem.value))
        srv.plan_request.problem_name = self._problem_name

        self._plan_pddl_one_shot_client_srv.wait_for_service()
        future = self._plan_pddl_one_shot_client_srv.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)
        plan_result = future.result().plan_result
        self._problem = self.get_problem()

        self._objects = {self._problem.all_objects[i].name : self._problem.all_objects[i] for i in range(len(self._problem.all_objects))}
        self._fluents = {self._problem.fluents[i].name : self._problem.fluents[i] for i in range(len(self._problem.fluents))}
        self._actions = {self._problem.actions[i].name : self._problem.actions[i] for i in range(len(self._problem.actions))}
        self._plan = plan_result.plan.actions
        
    def finished_action_callback(self, action, params, result):
        """
        Handles the callback from an action client after an action was executed.

        official reference implementation: https://github.com/ros2/examples/blob/master/rclpy/actions/minimal_action_client/examples_rclpy_minimal_action_client/client.py
        
        :param action: The action (in UPF format) that was completed
        :param params: The params (in UPF format) of the completed action
        :param result: Returned result of the completed action, contains goal status as well as action specific fields (see https://docs.ros2.org/galactic/api/action_msgs/msg/GoalStatus.html for goal status ENUMS)
        """
        # GoalStatus.STATUS_SUCCEEDED -> 4 is the status for successfully completed action in the GoalStatus Message, 5 for canceled action (GoalStatus.STATUS_CANCELED)
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Completed action: " + action.action_name+"("+", ".join(params)+")")
            self.update_initial_state(self._actions[action.action_name], params)
        else:
            self.get_logger().info("Canceled action: " + action.action_name+"("+", ".join(params)+")")
        # TODO: add error handling

    def action_feedback_callback(self, action, params, feedback):
        """
        Handles the feedback received by an action client
        # TODO: add implementation
        
        """
        None

    def execute_plan(self, action_finished_future, plan_finished_future):
        """
        Executes the next action of the plan. If no next action is available -> plan is completed and terminate

        
        :param action_finished_future: future that should be set when an action is completed so next action in the plan can be started
        :param plan_finished_future: future that should be set when a plan is completed so next plan can be started
        """
        if len(self._plan) == 0:
            self.get_logger().info('Plan completed!')
            plan_finished_future.set_result("Finished")
            action_finished_future.set_result("NoAction")
            return
        self._current_action_future = action_finished_future
        action = self._plan.pop(0)
        actionName = action.action_name
        params = [x.symbol_atom[0] for x in action.parameters]
        self.get_logger().info("Next action: " + action.action_name+"("+", ".join(params)+")")

        try:
            action_client : CustomActionClient = self._action_client_map[actionName]
            
            action_client.send_action_goal(action, params, action_finished_future)
            self._current_action_client = action_client
        except KeyError:
            self.get_logger().info("Error! Received invalid action name")
        
        #test code -> delete later
        #self.add_goal(self._fluents['visited'](self._objects['myuav'],self._objects['waters1']))

        
    def replan(self, request, response):
        """
        Handles a replan request from a client. The function will:
        1. Cancel the currently executed action
        2. Extract the new plan from the request
        3. Start the new plan

        
        :param request: The request from the client (containing the plan to be executed)
        :param response: The response object that will be returned to the client
        
        :returns: response: response to the client (as defined in Replan.srv)
        """    
        self.get_logger().info("Replanning: ")
        # Cancel current action
        if self._current_action_client != None:
            self._current_action_client.cancel_action_goal()
            self.get_logger().info("Successfully canceled actions")
        # Get new plan
        self._plan = request.plan_result.plan.actions
        self.get_logger().info("New plan: ")
        for action in self._plan:
            params = [x.symbol_atom[0] for x in action.parameters]
            self.get_logger().info("action: " + action.action_name+"("+", ".join(params)+")") 
        # Start new plan -> don't call execute plan here (called in main loop instead)
        self._current_action_future.set_result("Finished")
        response.success = True
        return response
        
    def mission_callback(self,goal_handle):
        """
        Execute a mission (plan and execution of that plan) and return the status after completion

        
        :param ServerGoalHandle goal_handle: Goal handler of the Action Server

        :returns: result: result message sended back to the action client
        :rtype: Mission.Result
        """
        self.get_logger().info(f'Mission current status: {goal_handle.request.init_status}')
        self.launch_mission()
        goal_handle.succeed()
        result = Mission.Result()
        result.final_status = [0,1,0,0]
        return result

    def launch_mission(self):
        """
        Launch a computation and execution of the plan
        
        """
        mte = MultiThreadedExecutor()
        self.get_plan_from_pddl()
        plan_finished_future = Future(executor = mte)
        while plan_finished_future.done() == False:
            action_finished_future = Future(executor = mte)
            self.execute_plan(action_finished_future,plan_finished_future)
            rclpy.spin_until_future_complete(self.sub_node,action_finished_future,mte)

def main(args=None):
    rclpy.init(args=args)
    plan_executor_node = PlanExecutorNode()
    plan_executor_node.launch_mission()
    rclpy.spin(plan_executor_node)
    
    plan_executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
