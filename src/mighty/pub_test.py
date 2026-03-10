#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped, Point
import test
import py_mighty

class TrajPublisher(Node):

    def __init__(self):
        super().__init__('traj_publisher')
        self.dataset, self.traj_list = test.compute_trajectory()
        self.traj_pub = self.create_publisher(
            Marker,
            "new_traj",
            10
        )

        self.goal_pub = self.create_publisher(
            PointStamped,
            "python_point_E",
            10
        )
        self.timer = self.create_timer(0.1, self.publish_traj)

    def publish_traj(self):

        print("publish_traj")
        for i, traj in enumerate(self.traj_list):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()

            marker.ns = "optimized_traj"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.05

            marker.color.r = (i % 3 == 0)*1.0
            marker.color.g = (i % 3 == 1)*1.0
            marker.color.b = (i % 3 == 2)*1.0
            marker.color.a = 1.0

            for pt in traj:
                p = Point()
                p.x = float(pt.pos[0])
                p.y = float(pt.pos[1])
                p.z = float(pt.pos[2])
                marker.points.append(p)

            self.traj_pub.publish(marker)

            # goal = PointStamped()
            # goal.header.frame_id = "map"
            # goal.header.stamp = self.get_clock().now().to_msg()

            # goal.point.x = float(self.end_state.pos[0])
            # goal.point.y = float(self.end_state.pos[1])
            # goal.point.z = float(self.end_state.pos[2])

            # self.goal_pub.publish(goal)


def main():
    rclpy.init()
    node = TrajPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    print

if __name__ == '__main__':
    main()