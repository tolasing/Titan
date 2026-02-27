# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import time


class BlockingWaitsForTransform(Node):
    """
    Wait for a transform syncronously.

    This class is an example of waiting for transforms.
    This will block the executor if used within a callback.
    Coroutine callbacks should be used instead to avoid this.
    See :doc:`examples_tf2_py/async_waits_for_transform.py` for an example.
    """

    def __init__(self):
        super().__init__('example_blocking_waits_for_transform')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.target_transform_value=1.222

        #self._output_timer = self.create_timer(1.0, self.on_timer)
        self.on_()

    def on_(self):
        self._output_timer = self.create_timer(1.0, self.on_timer)



    def on_timer(self):
        tf_ = False
        while tf_==False:
            from_frame = 'pallet_loader'
            to_frame = 'chassis'
            self.get_logger().info('Waiting for transform from {} to {}'.format(from_frame, to_frame))
            try:
                # Block until the transform is available
                #self.get_logger().info(rclpy.time.Time())
                current_time = rclpy.time.Time()
                self.get_logger().info(f'Current time: {current_time.to_msg().sec}.{current_time.to_msg().nanosec}')
                time.sleep(1.0)
                when = rclpy.time.Time()
                transform_stamped=self._tf_buffer.lookup_transform(
                    to_frame, from_frame, when, timeout=Duration(seconds=1.0))
                z_translation = transform_stamped.transform.translation.z
                self.get_logger().info(f'Transform translation z: {z_translation}')
                
                if self.transform_meets_criteria(transform_stamped):
                    self.get_logger().info("Target transform value reached.")
                    tf_=True
                    self._output_timer.cancel()
                    return
                
            except LookupException:
                self.get_logger().info('transform not ready')
        
    def transform_meets_criteria(self, transform_stamped: TransformStamped) -> bool:
        # Replace this with your specific criteria for the transform
        return abs(transform_stamped.transform.translation.z - self.target_transform_value) < 1.00


def main():
    from rclpy.executors import MultiThreadedExecutor

    rclpy.init()
    node = BlockingWaitsForTransform()
    # this node blocks in a callback, so a MultiThreadedExecutor is required
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    executor.shutdown()
    rclpy.shutdown()

if __name__=='__main__':
    main()