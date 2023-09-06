import rclpy
from rclpy.node import Node
from rclpy.action import CancelResponse, GoalResponse

from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger, SetBool

class AnafiInterfaceNode(Node):
    """
    Class description
    
    """

    def __init__(self):
        """
        Constructor method
        """
        super().__init__('anafi_interface')

        #declare params
        self.declare_parameter('drone_prefix', rclpy.Parameter.Type.STRING)

        self._drone_prefix = self.get_parameter('drone_prefix').get_parameter_value().string_value


        # create action servers
        self._fly_action_server = ActionServer(self, NavigateToPose, self._drone_prefix + "navigate_to_pose", self.execute_callback)
        self._takeoff_action_server = ActionServer(self, NavigateToPose,
                                                   self._drone_prefix + "takeoff",
                                                   self.execute_takeoff_callback,
                                                   goal_callback=self.goal_callback,
                                                   cancel_callback=self.cancel_callback)
        self._landing_action_server = ActionServer(self, NavigateToPose, self._drone_prefix + "landing", self.execute_callback)

        # create clients
        self._arm_drone_client = self.create_client(SetBool, '/anafi/drone/arm')
        self._drone_takeoff_client = self.create_client(Trigger, '/anafi/drone/takeoff')

        
    def execute_callback(self, goal_handle):
        None

    def goal_callback(self, goal_request):
        # Accepts or rejects a client request to begin an action
        self.get_logger().info('Received goal request')
        self.goal = goal_request
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        # Accepts or rejects a client request to cancel an action
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_fly_callback(self, goal_handle):
        None

    def execute_takeoff_callback(self, goal_handle):
        srv = SetBool.Request()
        future = self._arm_drone_client.call_async(srv)
        rclpy.spin_until_future_complete(self, future)

        srv = Trigger.Request()
        future = self._drone_takeoff_client.call_async(srv)
        rclpy.spin_until_future_complete(self, future)
        goal_handle.succeed()

        result = NavigateToPose.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    anafi_interface_node = AnafiInterfaceNode()
    rclpy.spin(anafi_interface_node)
    
    anafi_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
