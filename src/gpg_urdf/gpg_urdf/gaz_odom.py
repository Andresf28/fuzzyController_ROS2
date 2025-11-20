import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')

        self.sub = self.create_subscription(
            PoseArray,
            '/world/default/pose/info',
            self.callback,
            1
        )

        self.pub = self.create_publisher(Odometry, '/odom_gz', 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.robot_index = 1

    def callback(self, msg):
        if len(msg.poses) <= self.robot_index:
            return

        pose = msg.poses[self.robot_index]

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'     
        odom.child_frame_id = 'base_link'

        odom.pose.pose = pose

        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = GroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
