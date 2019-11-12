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

from geometry_msgs.msg import PoseStamped


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('move_fleet')
    publisher = node.create_publisher(PoseStamped, 'mir_fleet_manager/waypoint_goal')

    def timer_callback():
        msg = PoseStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = 17.0
        msg.pose.position.y = 10.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        publisher.publish(msg)

    timer_period = 0.5  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    # Destroy the timer attached to the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
