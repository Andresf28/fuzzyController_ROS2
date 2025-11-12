import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
import tf2_ros
import math
import tf2_geometry_msgs
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from gpg_urdf.GA_rules import FuzzyAvoider, FuzzyNavigator
import numpy as np
import time

class GAnavigation(Node):
    def __init__(self):
        super().__init__('ga_navigation')

        chrom = [3, 2, 1, 3, 2, 1, 3, 1, 1, 2, 1, 1,
         2, 4, 1, 2, 2, 0, 3, 2, 1, 2, 2, 0, 3, 1, 1, 0, 3, 0, 3, 4, 1, 3, 0, 1, 0, 4, 0, np.float64(0.0), 3, 1, 3, 3, 1, 1, 0, 0, 0, 1, 0, 3, 1, 1, 2, 1, 1, 3, 1, 1, 3, 0, np.float64(1.0), 1, 0, 0, 0, 3, 0, 0, 3, 1, 2, 0, 0, 0, 4, 1, 2, 4, 1, 3, 3, 1, 1, 3, 0, 0, 0, 0, 3, np.float64(1.0), 1, 1, 0, 1, 2, 0, 1, 0, 1, 0, 2, 0, 0, 0, 3, 0, 1, 2, 1, 3, 3, 1, 2, 2, 1, 0, 4, 0, 3, 0, 1, 2, 3, 0, 0, 2, 1, 0, 2, 0, 3, 2, 0, 1, 3, 0, 1, 3, np.float64(0.0), 2, 3, 0, 3, 2, 1, 3, 4, 1, 2, 2, 1, 2, 3, 1, 2, 2, 0, 0, 4, 0, 0, 2, 0, 1, 4, 1, 3, 2, 0, 1, 2, 0, 1, 1, 0, 1, 2, 1, 1, 4, 0, 3, 1, 0, 2, 4, 0, 0, 3, 1, 3, 3, 0, np.float64(1.0), 0, 1, 2, 0, 1, 2, 0, 1, 0, 1, 1, 3, 0, 1, 1, 3, 0, 1, 4, 0, 0, 4, 1, 2, 2, 0, 0, 1, 0, 3, 4, 0, 3, 4, 1, 2, 3, 0, 0, 2, 1, 3, 0, 1, 1, 4, 1, 1, 3, 0, 2, 1, 0, 2, 4, 1]

        fuzzy_nav = FuzzyNavigator()
        fuzzy_avoid = FuzzyAvoider()

        self.nav_sim = fuzzy_nav.build_nav_rules_ga(chrom[:60])
        self.avoid_sim = fuzzy_avoid.build_avoid_rules_ga(chrom[60:])

        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goalPose = self.create_subscription(PoseStamped,'/goal_pose', self.goalCallback,10)
        self.robotPos = self.create_subscription(Odometry,'/diff_drive_controller/odom', self.odomCallback,10)
        
        self.velPub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        self.goal_pose = None
        self.lidar_distance = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_update_time = time.time()

    def goalCallback(self, msg):
        self.goal_pose = msg

    def lidar_callback(self, msg):
        self.lidar_distance = msg
    
    def odomCallback(self, msg):
        if not self.goal_pose or not self.lidar_distance:
            return

        current_time = time.time()
        if current_time - self.last_update_time < 0.1:
            return
        self.last_update_time = current_time

        try:
            ranges = np.array(self.lidar_distance.ranges)
            ranges = np.nan_to_num(ranges, nan=self.lidar_distance.range_max, posinf=self.lidar_distance.range_max)

            num_points = len(ranges)
            angles = np.linspace(self.lidar_distance.angle_min, self.lidar_distance.angle_max, num_points)
            angles_deg = np.degrees(angles)

            front_mask = (angles_deg > -10) & (angles_deg < 10)
            left_mask = (angles_deg > 30) & (angles_deg < 90)
            right_mask = (angles_deg < -30) & (angles_deg > -90)

            front_dist = np.min(ranges[front_mask])
            left_dist = np.min(ranges[left_mask])
            right_dist = np.min(ranges[right_mask])

            goal = PoseStamped()
            goal.header.frame_id = self.goal_pose.header.frame_id
            goal.header.stamp = rclpy.time.Time().to_msg()
            goal.pose = self.goal_pose.pose

            goal_bl = self.tf_buffer.transform(goal, "base_link")
            
            dx = goal_bl.pose.position.x
            dy = goal_bl.pose.position.y
            
            distance = math.hypot(dx, dy)
            angle_error = math.atan2(dy, dx)

            try:
                self.nav_sim.input['dist_goal'] = distance
                self.nav_sim.input['angle_error'] = angle_error
                self.nav_sim.compute()
                v_nav = self.nav_sim.output['v_nav']
                w_nav = self.nav_sim.output['w_nav']

            except:
                self.get_logger().info(f"Excepcion nav_sim")
                pass

            try:
                self.avoid_sim.input['d_front'] = float(front_dist)
                self.avoid_sim.input['d_left'] = float(left_dist)
                self.avoid_sim.input['d_right'] = float(right_dist)

                self.avoid_sim.compute()
                v_avoid = self.avoid_sim.output['v_avoid']
                w_avoid = self.avoid_sim.output['w_avoid']

            except:
                self.get_logger().info(f"Excepcion avoid_sim")
                pass

            d_min = min(front_dist, left_dist, right_dist)
            alpha = np.clip((1.5 - d_min)/1.0, 0, 1)

            v_final = (1 - alpha)*v_nav + alpha*v_avoid
            w_final = (1 - alpha)*w_nav + alpha*w_avoid

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = float(v_final)
            twist_msg.twist.angular.z = float(w_final)
            
            if distance < 0.1:
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
            
            self.velPub.publish(twist_msg)
            self.get_logger().info(f"dist={distance:.2f}, angle_err={angle_error:.2f}, v={v_nav:.2f}, w={w_nav:.2f}")
        
        
        except Exception as e:
            self.get_logger().info(f"Excepcion general: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = GAnavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
