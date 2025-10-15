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


class FuzzyNavigator:
    def __init__(self):
        # Definición de universos
        self.dist_goal = ctrl.Antecedent(np.arange(0, 21, 0.1), 'dist_goal')
        self.angle_error = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.01), 'angle_error')
        self.v_nav = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_nav')
        self.w_nav = ctrl.Consequent(np.arange(-1, 1.1, 0.01), 'w_nav')

        # Funciones de membresía
        self.dist_goal['Very_near'] = fuzz.trimf(self.dist_goal.universe, [0, 0, 4])
        self.dist_goal['Near'] = fuzz.trimf(self.dist_goal.universe, [0, 4, 8])
        self.dist_goal['Medium'] = fuzz.trimf(self.dist_goal.universe, [4, 8, 12])
        self.dist_goal['Far'] = fuzz.trimf(self.dist_goal.universe, [8, 12, 16])
        self.dist_goal['Very_far'] = fuzz.trimf(self.dist_goal.universe, [12, 16, 20])

        self.angle_error['Left_big'] = fuzz.trimf(self.angle_error.universe, [-np.pi, -np.pi/2, -np.pi/4])
        self.angle_error['Left_small'] = fuzz.trimf(self.angle_error.universe, [-np.pi/4, -np.pi/8, 0])
        self.angle_error['Zero'] = fuzz.trimf(self.angle_error.universe, [-np.pi/12, 0, np.pi/12])
        self.angle_error['Right_small'] = fuzz.trimf(self.angle_error.universe, [0, np.pi/8, np.pi/4])
        self.angle_error['Right_big'] = fuzz.trimf(self.angle_error.universe, [np.pi/4, np.pi/2, np.pi])

        self.v_nav['Low'] = fuzz.trimf(self.v_nav.universe, [0, 0, 0.2])
        self.v_nav['Medium'] = fuzz.trimf(self.v_nav.universe, [0.2, 0.5, 0.8])
        self.v_nav['High'] = fuzz.trimf(self.v_nav.universe, [0.5, 0.8, 1.0])

        self.w_nav['Left'] = fuzz.trimf(self.w_nav.universe, [-1, -0.5, -0.2])
        self.w_nav['Left_small'] = fuzz.trimf(self.w_nav.universe, [-0.3, -0.15, 0])
        self.w_nav['Zero'] = fuzz.trimf(self.w_nav.universe, [-0.01, 0, 0.01])
        self.w_nav['Right_small'] = fuzz.trimf(self.w_nav.universe, [0, 0.15, 0.3])
        self.w_nav['Right'] = fuzz.trimf(self.w_nav.universe, [0.2, 0.5, 1])

        # Reglas
        rules_nav = [
            ctrl.Rule(self.dist_goal['Very_far'] & self.angle_error['Zero'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Zero'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Zero'], (self.v_nav['Medium'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Zero'], (self.v_nav['Low'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Zero'], (self.v_nav['Low'], self.w_nav['Zero'])),
            #------------------
            ctrl.Rule(self.dist_goal['Very_far'] & self.angle_error['Right_big'], (self.v_nav['High'], self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Right_big'], (self.v_nav['High'], self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Right_big'], (self.v_nav['Medium'], self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Right_big'], (self.v_nav['Low'], self.w_nav['Right'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Right_big'], (self.v_nav['Low'], self.w_nav['Right'])),
            #------------------
            ctrl.Rule(self.dist_goal['Very_far'] & self.angle_error['Right_small'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Right_small'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Right_small'], (self.v_nav['Medium'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Right_small'], (self.v_nav['Low'], self.w_nav['Right_small'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Right_small'], (self.v_nav['Low'], self.w_nav['Right_small'])),
            #------------------
            ctrl.Rule(self.dist_goal['Very_far'] & self.angle_error['Left_big'], (self.v_nav['High'], self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Left_big'], (self.v_nav['High'], self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Left_big'], (self.v_nav['Medium'], self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Left_big'], (self.v_nav['Low'], self.w_nav['Left'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Left_big'], (self.v_nav['Low'], self.w_nav['Left'])),
            #------------------
            ctrl.Rule(self.dist_goal['Very_far'] & self.angle_error['Left_small'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Far'] & self.angle_error['Left_small'], (self.v_nav['High'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Medium'] & self.angle_error['Left_small'], (self.v_nav['Medium'], self.w_nav['Zero'])),
            ctrl.Rule(self.dist_goal['Near'] & self.angle_error['Left_small'], (self.v_nav['Low'], self.w_nav['Left_small'])),
            ctrl.Rule(self.dist_goal['Very_near'] & self.angle_error['Left_small'], (self.v_nav['Low'], self.w_nav['Left_small'])),
        ]

        self.nav_ctrl = ctrl.ControlSystem(rules_nav)
        self.nav_sim = ctrl.ControlSystemSimulation(self.nav_ctrl)

    def compute(self, dist_goal_val, angle_error_val):
        self.nav_sim_local = ctrl.ControlSystemSimulation(self.nav_ctrl)
        self.nav_sim.input['dist_goal'] = dist_goal_val
        self.nav_sim.input['angle_error'] = angle_error_val
        self.nav_sim.compute()
        return self.nav_sim.output['v_nav'], self.nav_sim.output['w_nav']


class FuzzyAvoider:
    def __init__(self):
        self.d_front = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_front')
        self.d_left = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_left')
        self.d_right = ctrl.Antecedent(np.arange(0, 13, 0.1), 'd_right')
        self.v_avoid = ctrl.Consequent(np.arange(0, 1.1, 0.01), 'v_avoid')
        self.w_avoid = ctrl.Consequent(np.arange(-1.57, 1.57, 0.05), 'w_avoid')

        for d in [self.d_front, self.d_left, self.d_right]:
            d['Near'] = fuzz.trimf(d.universe, [0, 0, 0.7])
            d['Medium'] = fuzz.trimf(d.universe, [0.5, 2, 6])
            d['Far'] = fuzz.trimf(d.universe, [2, 6, 12])

        self.v_avoid['Low'] = fuzz.trimf(self.v_avoid.universe, [0, 0, 0.4])
        self.v_avoid['Medium'] = fuzz.trimf(self.v_avoid.universe, [0.2, 0.5, 0.8])
        self.v_avoid['High'] = fuzz.trimf(self.v_avoid.universe, [0.5, 1.0, 1.0])

        self.w_avoid['Left'] = fuzz.trimf(self.w_avoid.universe, [-1, -0.5, 0])
        self.w_avoid['Zero'] = fuzz.trimf(self.w_avoid.universe, [-0.01, 0, 0.01])
        self.w_avoid['Right'] = fuzz.trimf(self.w_avoid.universe, [0, 0.5, 1])

        rules_avoid = [
            ctrl.Rule(self.d_front['Near'] & self.d_left['Far'], (self.v_avoid['Low'], self.w_avoid['Right'])),
            ctrl.Rule(self.d_front['Near'] & self.d_right['Far'], (self.v_avoid['Low'], self.w_avoid['Left'])),
            ctrl.Rule(self.d_left['Near'], self.w_avoid['Right']),
            ctrl.Rule(self.d_right['Near'], self.w_avoid['Left']),
            ctrl.Rule(self.d_front['Far'] & self.d_left['Far'] & self.d_right['Far'], (self.v_avoid['High'], self.w_avoid['Zero']))
        ]

        self.avoid_ctrl = ctrl.ControlSystem(rules_avoid)
        self.avoid_sim = ctrl.ControlSystemSimulation(self.avoid_ctrl)

    def compute(self, d_front_val, d_left_val, d_right_val):
        self.avoid_sim = ctrl.ControlSystemSimulation(self.avoid_ctrl)
        self.avoid_sim.input['d_front'] = d_front_val
        self.avoid_sim.input['d_left'] = d_left_val
        self.avoid_sim.input['d_right'] = d_right_val
        self.avoid_sim.compute()
        return self.avoid_sim.output['v_avoid'], self.avoid_sim.output['w_avoid']



class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Call fuzzy controller:
        self.fuzzy_nav = FuzzyNavigator()
        self.fuzzy_avoid = FuzzyAvoider()

        # subscriptions:
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goalPose = self.create_subscription(PoseStamped,'/goal_pose', self.goalCallback,10)
        self.robotPos = self.create_subscription(Odometry,'/diff_drive_controller/odom', self.odomCallback,10)
        
        #publishers:
        self.velPub = self.create_publisher(TwistStamped, '/diff_drive_controller/cmd_vel', 10)
        
        # Initialize parameters:
        self.goal_pose = None
        self.lidar_distance = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Control timer
        #self.timer = self.create_timer(0.1, self.control_loop)

    def goalCallback(self, msg):
        self.goal_pose = msg

    def lidar_callback(self, msg):
        self.lidar_distance = msg
    
    def odomCallback(self, msg):
        if not self.goal_pose or not self.lidar_distance:
            return

        try:
            # ---------------- Lidar sensor ----------------
            ranges = np.array(self.lidar_distance.ranges)

            # Define max distance (inf and nan)
            ranges = np.nan_to_num(ranges, nan=self.lidar_distance.range_max, posinf=self.lidar_distance.range_max)

            num_points = len(ranges)
            angles = np.linspace(self.lidar_distance.angle_min, self.lidar_distance.angle_max, num_points)

            # dregrees
            angles_deg = np.degrees(angles)

            # Define angles for mask
            front_mask = (angles_deg > -10) & (angles_deg < 10)
            left_mask = (angles_deg > 30) & (angles_deg < 90)
            right_mask = (angles_deg < -30) & (angles_deg > -90)

            # Get the distance (min for each mask)
            front_dist = np.min(ranges[front_mask])
            left_dist = np.min(ranges[left_mask])
            right_dist = np.min(ranges[right_mask])

            self.get_logger().info(f"Front: {front_dist:.2f} m | Left: {left_dist:.2f} m | Right: {right_dist:.2f} m")

            # ---------------- goal pose ----------------
        
            goal = PoseStamped()
            goal.header.frame_id = self.goal_pose.header.frame_id
            goal.header.stamp = rclpy.time.Time().to_msg()
            goal.pose = self.goal_pose.pose

            #self.get_logger().info(f"TF from {goal.header.frame_id} to base_link exists? {self.tf_buffer.can_transform('base_link', goal.header.frame_id, rclpy.time.Time())}")

            
            goal_bl = self.tf_buffer.transform(goal, "base_link")
            
            dx = goal_bl.pose.position.x
            dy = goal_bl.pose.position.y
            
            distance = math.hypot(dx, dy)
            angle_error = math.atan2(dy, dx)

            v_nav, w_nav = self.fuzzy_nav.compute(distance, angle_error)
            #xw, w_avoid = self.fuzzy_avoid.compute(front_dist, left_dist, right_dist)

            d_min = min(front_dist, left_dist, right_dist)
            alpha = np.clip((1.0 - d_min)/1.0, 0, 1)

            #v_final = (1 - alpha)*v_nav + alpha*xw
            #w_final = (1 - alpha)*w_nav + alpha*w_avoid

            twist_msg = TwistStamped()
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            twist_msg.twist.linear.x = float(v_nav)  #anres era v_final
            twist_msg.twist.angular.z = float(w_nav)
            
            if distance < 0.1:
                twist_msg.twist.linear.x = 0.0
                twist_msg.twist.angular.z = 0.0
            
            self.velPub.publish(twist_msg)
            
            self.get_logger().info(f"dist={distance:.2f}, angle_err={angle_error:.2f}, v={v_nav:.2f}, w={w_nav:.2f}")
        
        except Exception as e:
            self.get_logger().warn(f"Can´t tranform to base_link: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
