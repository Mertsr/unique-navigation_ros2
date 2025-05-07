#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time

class AckermannSteering(Node):
    def __init__(self):
        super().__init__('ackermann_steering')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)

        self.goals = [
            (-3.256469249725342, 1.0977195739746094),  # 51
            (-16.964181900024414, 1.85830535888671875),  # 4
            (-32.28238296508789, 1.0202598571777344),  # 5
            (-46.88553237915039, -0.7234048843383789),  # 62
            (-48.799068450927734, 2.2548532485961914),  # 41
            (-57.84351348876953, -4.594797134399414),  # 42
            (-57.48166275024414, -8.770397186279297),  # 43
            (-58.05434799194336, -33.403563537597656),  # 6
            (-60.35447692871094, -33.85881690979004),  # 69
            (-60.909791564941406, -36.14098358154297),  # 70
            (-60.55547692871094, -39.64457321166992),  # 71
            (-57.48166275024414, -40.17243957519531),  # 37
            (-58.88736343383789, -46.97602081298828),  # 45
            (-53.271270751953125, -56.25480270385742),  # 8
            (-38.62897491455078, -56.62525177001953),  # 36
            (-25.00440979003906, -56.80315399169922),  # 48
            (-22.667098999023438, -49.56938171386719),  # 24
            (-18.52264404296875, -43.671268463134766),  # 68
            (-20.05948257446289, -32.27597427368164),  # 65
            (-24.44552993774414, -28.887008666992188),  # 40
            (-21.252763748168945, -27.500890731811523),  # 75
            (-19.817148208618164, -25.227603912353516),  # 76
            (-20.218708038330078, -23.255977630615234),  # 77
            (-25.31963348388672, -17.499862670898438),  # 19
            (-18.142765045166016, -15.177249908447266),  # 20
            (-10.976402282714844, -16.605083465576172),  # 3
            (-5.187399387359619, -20.187776565551758),  # 1
            (-1.2490692138671875, -23.06100082397461),  # 61
            (5.317310333251953, -23.857114791870117),  # 29
            (13.119555473327637, -15.201168060302734),  # 56
            (17.119555473327637, -16.201168060302734)  # 56
        ]
        self.orientations = [
            (0.9999999999972611, 2.3404744010116256e-06),  # 51
            (0.9999129331711489, 0.013195684031887056),  # 4
            (0.9998700001379174, 0.01612398288888829),  # 5
            (0.014920740839417527, 0.9998886795502802),  # 62
            (-0.9998574155681166, 0.01688634162412122),  # 41
            (-0.7080975636743657, 0.7061146084867722),  # 42
            (-0.7080975636743657, 0.7061146084867722),  # 43
            (-0.7291953081077596, 0.6843056353952008),  # 6
            (-0.7895702939784931, 0.6136601265087345),  # 69
            (-0.7168366885275501, 0.6972411074950013),  # 70
            (-0.5214286578580642, 0.8532948814825609),  # 71
            (0.7008144623610688, 0.7133435983770836),  # 37
            (-0.7036382572375567, 0.7105583740634502),  # 45
            (-0.018510061210819207, 0.9998286741407109),  # 8
            (0.9989728036992419, 0.04531376688464558),  # 36
            (-0.04895512108899282, 0.9988009792341825),  # 48
            (0.6846876602843075, 0.7288366125918763),  # 24
            (0.49570165470423594, 0.8684928724655617),  # 68
            (0.9665970614423981, 0.2563008404413079),  # 65
            (0.6699266590975517, 0.7424272835977896),  # 40
            (0.41066677728618917, 0.9117855000126814),  # 75
            (0.7175059949062503, 0.6965523291710336),  # 76
            (0.9298251484221924, 0.3680016214117104),  # 77
            (0.6742482591420738, 0.7385047630475262),  # 19
            (-0.9999999091052009, 0.0004263679044149339),  # 20
            (0.025887811794878868, 0.9996648544389629),  # 3
            (0.7100716878118494, 0.7041293902174738),  # 1
            (0.9988273158318313, 0.048414802986061406),  # 61
            (-0.02122826101220333, 0.9997746550770318),  # 29
            (0.010168777780201815, 0.0000010000000000),  # 56
            (0.010168777780201815, 0.0000010000000000)  # 56
        ]
        self.current_goal_index = 0
        self.waiting = False  # Yeni değişken: bekleme durumu

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laser_data = []
        self.obstacle_detected = False
        self.timer = self.create_timer(0.1, self.navigate_to_goal)
        self.avoid_obstacle_mode = False
        self.circular_motion_timer = 0
        self.wheelbase = 1.0  # Araç dingil mesafesi

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.euler_from_quaternion(orientation_q)

    def scan_callback(self, msg):
        self.laser_data = msg.ranges

    def euler_from_quaternion(self, quat):
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = +1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def navigate_to_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals reached!')
            return

        # Check if obstacle avoidance is disabled for the current goal index
        if self.current_goal_index in [8, 9, 10, 20, 21, 22]:  # 9., 10. ve 11. hedefler için (indeksler 8, 9, 10)
            self.avoid_obstacle_mode = False
        else:
            if self.is_obstacle_detected():
                if self.current_goal_index < len(self.goals) - 1:
                    self.avoid_obstacle_mode = True
                else:
                    self.avoid_obstacle_mode = False

        if self.avoid_obstacle_mode:
            self.avoid_obstacle()
        else:
            self.move_to_goal()

    def is_obstacle_detected(self):
        if not self.laser_data:
            return False
        front_ranges = self.laser_data[len(self.laser_data)//2 - 15 : len(self.laser_data)//2 + 15]
        min_distance = min(front_ranges)
        self.obstacle_detected = min_distance < 8.0  # Engel mesafesi (8.0 metre)
        return self.obstacle_detected

    def avoid_obstacle(self):
        if self.is_path_clear():
            self.move_to_goal()
        else:
            if self.circular_motion_timer < 100:  # 10 saniye dairesel hareket (100 * 0.1 saniye)
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.5  # İleriye doğru hareket
                cmd_msg.angular.z = 0.5  # Dairesel hareket için daha geniş dönme
                self.publisher_.publish(cmd_msg)
                self.circular_motion_timer += 1
            else:
                self.reset_timers()
                self.avoid_obstacle_mode = False

    def is_path_clear(self):
        front_ranges = self.laser_data[len(self.laser_data)//2 - 30 : len(self.laser_data)//2 + 30]
        min_distance = min(front_ranges)
        return min_distance >= 8.0

    def reset_timers(self):
        self.circular_motion_timer = 0

    def stop_robot(self):
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.publisher_.publish(cmd_msg)

    def move_to_goal(self):
        if self.waiting:  # Bekleme modundaysa, beklemeye devam et
            return
        
        target_x, target_y = self.goals[self.current_goal_index]
        target_orientation_z, target_orientation_w = self.orientations[self.current_goal_index]
        distance = math.sqrt((target_x - self.current_x) ** 2 + (target_y - self.current_y) ** 2)
        angle_to_goal = math.atan2(target_y - self.current_y, target_x - self.current_x)

        angular_error = angle_to_goal - self.current_yaw
        if angular_error > math.pi:
            angular_error -= 2 * math.pi
        elif angular_error < -math.pi:
            angular_error += 2 * math.pi

        if distance > 1.75:  # Hedef alanı (1.75 metre yarıçap, 3.5 metre çap)
            cmd_msg = Twist()
            if abs(angular_error) > 0.1:
                cmd_msg.linear.x = 0.5  # Dönüş yaparken daha yavaş hareket
                cmd_msg.angular.z = 1.0 * angular_error  # Daha geniş dönüş yap
            else:
                cmd_msg.linear.x = 2.0  # Daha hızlı ileri hareket
                cmd_msg.angular.z = 0.2 * angular_error  # Daha az dön
            self.publisher_.publish(cmd_msg)
        else:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} reached!')
            self.stop_robot()
            if self.current_goal_index in [10, 22]:
                self.waiting = True
                self.get_logger().info(f'Waiting for 30 seconds at goal {self.current_goal_index + 1}...')
                self.timer = self.create_timer(30.0, self.resume_navigation)
            else:
                self.current_goal_index += 1

    def resume_navigation(self):
        self.waiting = False
        self.get_logger().info('Resuming navigation...')
        self.current_goal_index += 1
        self.timer.cancel()  # Bekleme zamanlayıcısını iptal et

    def ackermann_steer(self, linear_velocity, angular_velocity):
        cmd_msg = Twist()
        cmd_msg.linear.x = linear_velocity
        if angular_velocity != 0:
            cmd_msg.angular.z = linear_velocity * math.tan(angular_velocity) / self.wheelbase
        else:
            cmd_msg.angular.z = 0
        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    ackermann_steering = AckermannSteering()
    rclpy.spin(ackermann_steering)
    ackermann_steering.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

