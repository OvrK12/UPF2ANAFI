import rclpy
from rclpy.node import Node
from rclpy.action import CancelResponse, GoalResponse, ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from geopy.distance import geodesic

from anafi_ros_interfaces.msg import MoveToCommand
from geometry_msgs.msg import Pose, PointStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Float32


class AnafiBridgeNode(Node):
    """
    Class description
    
    """

    def __init__(self):
        """
        Constructor method
        """
        super().__init__('anafi_offboard')

        #declare params
        self.declare_parameter('drone_prefix', rclpy.Parameter.Type.STRING)

        self._drone_prefix = self.get_parameter('drone_prefix').get_parameter_value().string_value
        self.sub_node = rclpy.create_node('sub_node_' + self._drone_prefix[0:-1], use_global_arguments=False)


        self._altitude = 0.0
        self._position = (0.0,0.0)
        # create publishers
        self.pub_move_to = self.create_publisher(MoveToCommand, '/anafi/drone/moveto', qos_profile_sensor_data)

        # create subscribers
        self.sub_altitude = self.create_subscription(Float32,
                                                     '/anafi/drone/altitude',
                                                     self.altitude_changed_callback,
                                                     qos_profile_sensor_data)
        self.sub_position = self.create_subscription(PointStamped,
                                                     '/anafi/drone/position',
                                                     self.position_changed_callback,
                                                     qos_profile_sensor_data)

        # create action servers
        self._fly_action_server = ActionServer(self, NavigateToPose,
                                               self._drone_prefix + "navigate_to_pose",
                                               self.execute_fly_callback,
                                               goal_callback=self.goal_callback,
                                               cancel_callback=self.cancel_callback)
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
    
    def altitude_changed_callback(self,msg):
        self._altitude = msg.data
    
    def position_changed_callback(self,msg):
        self._position = (msg.point.x,msg.point.y)
        self.get_logger().info(f"position_changed_callback {self._position}")

    def execute_fly_callback(self, goal_handle):
        self.get_logger().info(f"in fly callback")
        msg = MoveToCommand()
        requestedPose: Pose = goal_handle.request.pose.pose
        msg.longitude = requestedPose.position.x
        msg.latitude = requestedPose.position.y
        msg.altitude = requestedPose.position.z
        msg.heading = 0.0
        msg.orientation_mode = 1 # 1 means heading to destination. Reference: https://developer.parrot.com/docs/olympe/arsdkng_move.html#olympe.enums.move.orientation_mode
        self.pub_move_to.publish(msg)

        distance_to_destination = geodesic(self._position,(msg.longitude,msg.latitude))
        altitude_dif = abs(self._altitude - msg.altitude)
        self.get_logger().info(f"distance_to_destination {distance_to_destination} altitude_dif {altitude_dif}")
        # loop until destination is reached
        while distance_to_destination > 5.0 or altitude_dif > 5.0 :
            rclpy.spin_once(self.sub_node,timeout_sec=1)
            distance_to_destination = geodesic(self._position,(msg.longitude,msg.latitude))
            altitude_dif = abs(self._altitude - msg.altitude)
            self.get_logger().info(f"spin {distance_to_destination}")
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result

    def execute_takeoff_callback(self, goal_handle):
        srv = SetBool.Request()

        future = self._arm_drone_client.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)

        srv = Trigger.Request()
        future = self._drone_takeoff_client.call_async(srv)
        rclpy.spin_until_future_complete(self.sub_node, future)

        # loop until take_off completed. Margin of 1.0, because parrot takes off 1m from the ground per default
        while self._altitude < 1.0:
            rclpy.spin_once(self.sub_node,timeout_sec=1)
            self.get_logger().info(f"spin {self._altitude}")
            
        goal_handle.succeed()
        result = NavigateToPose.Result()
        return result


def main(args=None):
    rclpy.init(args=args)
    anafi_offboard_node = AnafiBridgeNode()
    rclpy.spin(anafi_offboard_node)
    anafi_offboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
