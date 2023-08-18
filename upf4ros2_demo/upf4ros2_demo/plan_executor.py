import rclpy
import json

from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.node import Node
from rosidl_runtime_py import convert as RosMsgConverter
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from ament_index_python.packages import get_package_share_directory

from upf4ros2.ros2_interface_reader import ROS2InterfaceReader
from upf4ros2.ros2_interface_writer import ROS2InterfaceWriter
from upf_msgs import msg as msgs
from upf4ros2_demo.action_clients.take_off_action_client import TakeOffActionClient
from upf4ros2_demo.action_clients.land_action_client import LandActionClient
from upf4ros2_demo.action_clients.fly_action_client import FlyActionClient

from upf_msgs.action import (
    PDDLPlanOneShot
)

from upf_msgs.srv import (
    AddGoal,
    GetProblem,
    SetInitialValue,
    Replan
)

from upf_msgs.srv import PDDLPlanOneShot as PDDLPlanOneShotSrv

from upf4ros2_demo.action_clients.customaction_client import CustomActionClient
from upf4ros2_demo.action_clients.inspect_action_client import InspectActionClient
from rclpy.action import ActionServer
from upf4ros2_demo_msgs.action import Mission

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class PlanExecutorNode(Node):
    """
    Ros2 node used to create and execute a plan
    
    """

    def __init__(self):
        """
        Constructor method
        """
        super().__init__('plan_executor')

        
        self._problem = None
        self._actions = {}
        self._objects = {}
        self._fluents = {}
        self._plan = []
        self._current_action_future = None
        self._current_action_client = None
        
        callback_group = MutuallyExclusiveCallbackGroup()
        
        self.declare_parameter('drone_prefix', rclpy.Parameter.Type.STRING)
        self.declare_parameter('drone_id',rclpy.Parameter.Type.INTEGER)
        self._drone_id=self.get_parameter('drone_id').value#int(self.get_parameter('drone_id').get_parameter_value())
        self._problem_name = f'uav_problem{self._drone_id}'
        
        # Separate PDDL files for stochastic game
        self.declare_parameter('domain', '/pddl/monitor_domain_upf.pddl')
        self.declare_parameter('problem', f'/pddl/monitorproblem_{self._drone_id}.pddl')
        # self.declare_parameter('domain', '/pddl/uav_domain.pddl')
        # self.declare_parameter('problem', '/pddl/generated_uav_instance.pddl')
        

        self._domain = self.get_parameter('domain')
        self._problem = self.get_parameter('problem')
        self._drone_prefix = self.get_parameter('drone_prefix').get_parameter_value().string_value
        

        self._ros2_interface_writer = ROS2InterfaceWriter()
        self._ros2_interface_reader = ROS2InterfaceReader()
        
        self.sub_node = rclpy.create_node('sub_node_' + self._drone_prefix[0:-1], use_global_arguments=False)

        self._take_off_client = TakeOffActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._land_client = LandActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._fly_client = FlyActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        self._inspect_client = InspectActionClient(self.sub_node,self.action_feedback_callback,self.finished_action_callback, self._drone_prefix)
        
        self._current_action_client = None

        self._plan_pddl_one_shot_client = ActionClient(self, PDDLPlanOneShot, 'upf4ros2/action/planOneShotPDDL')
        
        self._get_problem = self.sub_node.create_client(GetProblem, 'upf4ros2/srv/get_problem')
        self._set_initial_value = self.sub_node.create_client(SetInitialValue, 'upf4ros2/srv/set_initial_value')
        self._plan_pddl_one_shot_client_srv = self.sub_node.create_client(PDDLPlanOneShotSrv, 'upf4ros2/srv/planOneShotPDDL')
        
        replan_server_name = 'upf4ros2/srv/' + self._drone_prefix + 'replan'
        self._replan = self.sub_node.create_service(Replan, replan_server_name, self.replan)
        
        self.mission_action_server = ActionServer(self,Mission, f'mission{self._drone_id}',self.mission_callback)
        
        
        
    def set_initial_value(self, fluent, object, value_fluent):
        """
        _summary_

        :param <type> fluent:<description>
        :param <type> object:<description>
        :param <type> value_fluent:<description>
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
        Updates the problem with all the effects from an action

        
        :param upf_msgs/Action action: the executed action with it's associated effects
        :param <type> parameters: <description>
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
    
    def add_goal(self, goal):
        """
        _summary_

        
        :param <type> goal (_type_): <description>
        """        
        srv = AddGoal.Request()
        srv.problem_name = self._problem_name
        upf_goal = msgs.Goal()
        upf_goal.goal = self._ros2_interface_writer.convert(goal)
        srv.goal.append(upf_goal)
        srv.drone_id = self._drone_prefix
        test = RosMsgConverter.message_to_ordereddict(srv)
        with open('result.json', 'w') as fp:
            json.dump(test, fp)
        self._add_goal.wait_for_service()
        future = self._add_goal.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)

        self.get_logger().info(f'Set new goal!')

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
    
    def get_plan_srv(self):
        """
        _summary_
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
        _summary_

        
        :param <type> action: <description>
        :param <type> params: <description>
        :param <type> result: <description>
        """
        # TODO: replace magic number -> 4 is the code for successfully completed action in NavigateToPose action format, 5 for canceled action
        if result == 4:
            self.get_logger().info("Completed action: " + action.action_name+"("+", ".join(params)+")")
            self.update_initial_state(self._actions[action.action_name], params)
        else:
            self.get_logger().info("Canceled action: " + action.action_name+"("+", ".join(params)+")")
        # TODO: add error handling

    # TODO: add implementation
    def action_feedback_callback(self, action, params, feedback):
        """
        _summary_

        
        :param <type> action: <description>
        :param <type> params: <description>
        :param <type> feedback: <description>
        """
        None

    def execute_plan(self, action_finished_future, plan_finished_future):
        """
        _summary_

        
        :param Future action_finished_future: <description>
        :param Future plan_finished_future: <description>
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

        if actionName == "take_off":
            self._take_off_client.send_action_goal(action, params, action_finished_future)
            self._current_action_client = self._take_off_client
        elif actionName == "land":
            self._land_client.send_action_goal(action, params, action_finished_future)
            self._current_action_client = self._land_client
        elif actionName == "fly":
            self._fly_client.send_action_goal(action, params, action_finished_future)
            self._current_action_client = self._fly_client
        else:
            self.get_logger().info("Error! Received invalid action name")
            self._inspect_client.send_action_goal(action, params, action_finished_future)
            self._current_action_client = self._inspect_client
        #test code -> delete later
        #self.add_goal(self._fluents['visited'](self._objects['myuav'],self._objects['waters1']))

        
    def replan(self, request, response):
        """
        _summary_

        
        :param <type> request: <description>
        :param <type> response: <description>
        
        :returns: response: <description>
        :rtype: <type>
        """    
        self.get_logger().info("Replanning: ")
        # Cancel current action
        if self._current_action_client != None:
            # TODO: make this call blocking
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
        # MultiThreadedExecutor to avoid deadlock
        mte = MultiThreadedExecutor()
        self.get_plan_srv()
        plan_finished_future = Future(executor=mte)
        while plan_finished_future.done() == False:
            action_finished_future = Future(executor=mte)
            self.execute_plan(action_finished_future,plan_finished_future)
            rclpy.spin_until_future_complete(self.sub_node,action_finished_future,mte)

def main(args=None):
    rclpy.init(args=args)
    #Multi threaded executor is used to avoid deadlock when multiple drone are used with replanning
    mte_out = MultiThreadedExecutor()
    plan_executor_node = PlanExecutorNode()
    mte_out.add_node(plan_executor_node)
    #plan_executor_node.launch_mission()
    mte_out.spin()
    # rclpy.spin(plan_executor_node)
    
    plan_executor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
