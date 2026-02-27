import rclpy
import time
from rclpy.action import ActionClient
from rclpy.node import Node

from prototype.action import GoTo,FoodMenu,Actuator


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self,Actuator, '/PalletActuator')

    def send_goal(self, msg):
        goal_msg = Actuator.Goal()
        goal_msg.position=1.0
        print("hello")
        self._action_client.wait_for_server()
        print('hi')
        
        send_goal_future=self._action_client.send_goal_async(goal_msg,feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self,send_goal_future)
        print(send_goal_future.result())
        goal_handle=send_goal_future.result()
        result_future=goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self,result_future)
        print(result_future.result().result.done)


    
    def feedback_callback(self,feedback_msg):
        feedback=feedback_msg.feedback
        self.get_logger().info(f'Feedback: x={feedback}')


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal([1.0])

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()