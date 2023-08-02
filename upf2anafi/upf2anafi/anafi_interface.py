import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose

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
        self._takeoff_action_server = ActionServer(self, NavigateToPose, self._drone_prefix + "takeoff", self.execute_callback)
        self._landing_action_server = ActionServer(self, NavigateToPose, self._drone_prefix + "landing", self.execute_callback)

        
    def execute_callback(self):
        None

def main(args=None):
    rclpy.init(args=args)
    anafi_interface_node = AnafiInterfaceNode()
    rclpy.spin(anafi_interface_node)
    
    anafi_interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
