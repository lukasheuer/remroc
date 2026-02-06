# Copyright (c) 2024 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/remroc/.
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


import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class RobotOdometryPublisher(Node):
        """
        This Node subscribes the tf tree in its respective namespace and uses the
        map -> odom transformation published by a localization algorithm like amcl.
        It subscripes to the "/odometry/filtered" topic which is published by a
        state-estimation algorithm like an ekf. It then uses the afore mentioned transform
        to publish the same message only in the map frame.
        """

        def __init__(self):
                super().__init__('robot_odometry_publisher')

                # The target frame to which the messages should be transformed
                self.declare_parameter('x', 0.0)
                self.x = self.get_parameter('x').value
                self.declare_parameter('y', 0.0)
                self.y = self.get_parameter('y').value
                self.declare_parameter('yaw', 0.0)
                self.yaw = self.get_parameter('yaw').value

                self.last_x = self.x
                self.last_yaw = self.yaw
                self.last_time = 0.0

                # Creating the publisher and subscriber to the respective topics
                self.subscriber_ = self.create_subscription(PoseStamped, 'robot_state', self.callback_function, 10)
                self.publisher_ = self.create_publisher(Odometry, 'odometry/filtered', 10)
                self.tf_broadcaster = TransformBroadcaster(self)

        def callback_function(self, msg):
                x, y = msg.pose.position.x, msg.pose.position.y

                # Translate
                dx = x - self.x
                dy = y - self.y

                # Rotate by -yaw0
                x_local = dx * math.cos(-self.yaw) - dy * math.sin(-self.yaw)
                y_local = dx * math.sin(-self.yaw) + dy * math.cos(-self.yaw)

                # Extract incoming orientation
                q = msg.pose.orientation
                _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                yaw_local = yaw - self.yaw

                q_local = quaternion_from_euler(0.0, 0.0, yaw_local)

                # Create Odometry message
                odom = Odometry()
                odom.header = msg.header
                odom.header.frame_id = 'odom'  # local frame
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = x_local
                odom.pose.pose.position.y = y_local
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = q_local[0]
                odom.pose.pose.orientation.y = q_local[1]
                odom.pose.pose.orientation.z = q_local[2]
                odom.pose.pose.orientation.w = q_local[3]

                time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                dt = time - self.last_time
                vx = (x_local - self.last_x) / dt
                v_yaw = (yaw_local - self.last_yaw) / dt
                if abs(v_yaw > 1):
                        v_yaw = 0.0

                # Twist can be left zero if unknown
                odom.twist.twist.linear.x = vx
                odom.twist.twist.angular.z = v_yaw

                self.last_x = x_local
                self.last_yaw = yaw_local
                self.last_time = time

                self.publisher_.publish(odom)

                t = TransformStamped()
                t.header.stamp = msg.header.stamp
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = x_local
                t.transform.translation.y = y_local
                t.transform.translation.z = 0.0
                t.transform.rotation.x = q_local[0]
                t.transform.rotation.y = q_local[1]
                t.transform.rotation.z = q_local[2]
                t.transform.rotation.w = q_local[3]
                self.tf_broadcaster.sendTransform(t)


def main(args=None):
        rclpy.init(args=args)

        robot_state_publisher = RobotOdometryPublisher()

        rclpy.spin(robot_state_publisher)

        robot_state_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
        main()
