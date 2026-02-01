#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# gazebo_autonomous_controller.py

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import LaserScan, Imu, FluidPressure
from uuv_sensor_ros_plugins_msgs.msg import DVL
from tf.transformations import euler_from_quaternion

class GazeboAutonomousController:
    def __init__(self):
        rospy.init_node('gazebo_autonomous_controller')

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_out', Twist, queue_size=1)

        # Subscribers
        self.dvl_sub = rospy.Subscriber('dvl_data', DVL, self.dvl_callback)
        from geometry_msgs.msg import TwistWithCovarianceStamped

        self.dvl_twist_sub = rospy.Subscriber('dvl_twist', TwistWithCovarianceStamped, self.dvl_twist_callback)

        self.imu_sub = rospy.Subscriber('imu_data', Imu, self.imu_callback)
        self.pressure_sub = rospy.Subscriber('pressure_data', FluidPressure, self.pressure_callback)

        from sensor_msgs.msg import Range

        self.dvl_sonar0_sub = rospy.Subscriber('dvl_sonar0', Range, self.dvl_sonar0_callback)
        self.dvl_sonar1_sub = rospy.Subscriber('dvl_sonar1', Range, self.dvl_sonar1_callback)
        self.dvl_sonar2_sub = rospy.Subscriber('dvl_sonar2', Range, self.dvl_sonar2_callback)
        self.dvl_sonar3_sub = rospy.Subscriber('dvl_sonar3', Range, self.dvl_sonar3_callback)


        self.current_depth = 0.0
        self.target_depth = -10.0
        self.current_altitude = 0.0

        self.obstacle_distances = {
            'front': float('inf'),
            'back': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.angular_velocity = [0.0, 0.0, 0.0]

        self.dvl_velocity = [0.0, 0.0, 0.0]
        self.dvl_altitude = 0.0
        self.dvl_valid = False

        self.depth_kp = 1.0
        self.depth_kd = 0.3
        self.altitude_kp = 0.5
        self.yaw_kp = 0.8

        self.min_altitude = 2.0
        self.obstacle_threshold = 3.0
        self.safe_distance = 5.0

        self.prev_depth_error = 0.0
        self.prev_time = rospy.Time.now()

        self.mission_state = "DIVE"
        self.target_yaw = 0.0

        self.waypoints = [
            [10.0, 0.0, -10.0],
            [10.0, 10.0, -10.0],
            [0.0, 10.0, -10.0],
            [0.0, 0.0, -10.0]
        ]
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 2.0

        self.control_timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("Gazebo Otonom Kontrol Sistemi başlatıldı!")
        rospy.loginfo("Hedef derinlik: {}m".format(self.target_depth))

    def dvl_callback(self, msg):
    # velocity alanı varsa kullan
        if hasattr(msg, 'velocity'):
            self.dvl_velocity = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
            self.dvl_valid = True
        else:
            self.dvl_valid = False

        # altitude kontrolü
        if hasattr(msg, 'altitude') and msg.altitude > 0:
            self.dvl_altitude = msg.altitude
            self.current_altitude = msg.altitude



    def dvl_twist_callback(self, msg):
        self.dvl_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]


    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.current_roll, self.current_pitch, self.current_yaw) = euler_from_quaternion(orientation_list)

        self.angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]

    def pressure_callback(self, msg):
        atmospheric_pressure = 101325.0
        water_density = 1025.0
        gravity = 9.81

        if msg.fluid_pressure > atmospheric_pressure:
            gauge_pressure = msg.fluid_pressure - atmospheric_pressure
            depth = gauge_pressure / (water_density * gravity)
            self.current_depth = -depth
        else:
            self.current_depth = 0.0

    def dvl_sonar0_callback(self, msg):
        distance = msg.range
        # Mesafe geçerli mi kontrol edelim
        if not np.isinf(distance) and not np.isnan(distance):
            self.obstacle_distances['front'] = distance


    def dvl_sonar1_callback(self, msg):
        distance = msg.range
        # Mesafe geçerli mi kontrol edelim
        if not np.isinf(distance) and not np.isnan(distance):
            self.obstacle_distances['right'] = distance

    def dvl_sonar2_callback(self, msg):
        distance = msg.range
        # Mesafe geçerli mi kontrol edelim
        if not np.isinf(distance) and not np.isnan(distance):
            self.obstacle_distances['back'] = distance

    def dvl_sonar3_callback(self, msg):
        distance = msg.range
        # Mesafe geçerli mi kontrol edelim
        if not np.isinf(distance) and not np.isnan(distance):
            self.obstacle_distances['left'] = distance

    def control_loop(self, event):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()

        if dt <= 0:
            return

        cmd = Twist()

        if self.mission_state == "DIVE":
            cmd = self.dive_control(dt)
        elif self.mission_state == "NAVIGATE":
            cmd = self.navigate_control(dt)
        elif self.mission_state == "HOVER":
            cmd = self.hover_control(dt)
        elif self.mission_state == "SURFACE":
            cmd = self.surface_control(dt)

        cmd = self.apply_safety_limits(cmd)
        self.cmd_vel_pub.publish(cmd)

        self.update_mission_state()
        self.prev_time = current_time

        if rospy.get_time() % 3 < 0.1:
            self.print_status()

    def dive_control(self, dt):
        cmd = Twist()

        depth_error = self.target_depth - self.current_depth
        depth_derivative = (depth_error - self.prev_depth_error) / dt
        cmd.linear.z = self.depth_kp * depth_error + self.depth_kd * depth_derivative

        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        cmd.angular.z = self.yaw_kp * yaw_error

        self.prev_depth_error = depth_error
        return cmd

    def navigate_control(self, dt):
        cmd = Twist()

        if self.current_waypoint_idx < len(self.waypoints):
            target_waypoint = self.waypoints[self.current_waypoint_idx]

            depth_error = target_waypoint[2] - self.current_depth
            cmd.linear.z = self.depth_kp * depth_error * 0.5

            dx = target_waypoint[0]
            dy = target_waypoint[1]
            target_yaw = math.atan2(dy, dx)

            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            cmd.angular.z = self.yaw_kp * yaw_error

            if abs(yaw_error) < 0.2:
                cmd.linear.x = 0.5
            else:
                cmd.linear.x = 0.1

        cmd = self.avoid_obstacles(cmd)
        return cmd

    def hover_control(self, dt):
        cmd = Twist()

        depth_error = self.target_depth - self.current_depth
        cmd.linear.z = self.depth_kp * depth_error * 0.3

        yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
        cmd.angular.z = self.yaw_kp * yaw_error * 0.5

        cmd.linear.x = -self.dvl_velocity[0] * 0.5
        cmd.linear.y = -self.dvl_velocity[1] * 0.5

        return cmd

    def surface_control(self, dt):
        cmd = Twist()

        if self.current_depth < -0.5:
            cmd.linear.z = 0.3
        else:
            cmd.linear.z = 0.0

        return cmd

    def avoid_obstacles(self, cmd):
        min_distance = min(self.obstacle_distances.values())

        if min_distance < self.obstacle_threshold:
            cmd.linear.x = 0.0

            if self.obstacle_distances['front'] == min_distance:
                cmd.linear.x = -0.2
                cmd.angular.z = 0.3
            elif self.obstacle_distances['right'] == min_distance:
                cmd.angular.z = 0.3
            elif self.obstacle_distances['left'] == min_distance:
                cmd.angular.z = -0.3
            elif self.obstacle_distances['back'] == min_distance:
                cmd.linear.x = 0.2

            rospy.logwarn("Engel tespit edildi! Minimum mesafe: {:.2f}m".format(min_distance))

        elif min_distance < self.safe_distance:
            cmd.linear.x = min(cmd.linear.x, 0.2)

        return cmd

    def apply_safety_limits(self, cmd):
        cmd.linear.x = max(-1.0, min(1.0, cmd.linear.x))
        cmd.linear.y = max(-1.0, min(1.0, cmd.linear.y))
        cmd.linear.z = max(-0.5, min(0.5, cmd.linear.z))
        cmd.angular.z = max(-0.5, min(0.5, cmd.angular.z))

        if self.current_altitude < self.min_altitude and self.current_altitude > 0:
            cmd.linear.z = max(0.0, cmd.linear.z)
            rospy.logwarn("Deniz tabanına çok yakın! Altitude: {:.2f}m".format(self.current_altitude))

        return cmd

    def update_mission_state(self):
        if self.mission_state == "DIVE":
            if abs(self.current_depth - self.target_depth) < 1.0:
                self.mission_state = "NAVIGATE"
                rospy.loginfo("Hedef derinliğe ulaşıldı, navigasyon moduna geçiliyor...")
        elif self.mission_state == "NAVIGATE":
            if self.current_waypoint_idx < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_idx]
                distance = math.sqrt(target[0]**2 + target[1]**2)
                if distance < self.waypoint_threshold:
                    self.current_waypoint_idx += 1
                    rospy.loginfo("Waypoint {} tamamlandı!".format(self.current_waypoint_idx))
            if self.current_waypoint_idx >= len(self.waypoints):
                self.mission_state = "HOVER"
                rospy.loginfo("Tüm waypoint'ler tamamlandı, hover moduna geçiliyor...")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def print_status(self):
        rospy.loginfo("="*50)
        rospy.loginfo("Görev Durumu: {}".format(self.mission_state))
        rospy.loginfo("Derinlik: {:.2f}m (Hedef: {:.2f}m)".format(self.current_depth, self.target_depth))
        rospy.loginfo("Altitude: {:.2f}m".format(self.current_altitude))
        rospy.loginfo("Yaw: {:.1f}°".format(math.degrees(self.current_yaw)))
        rospy.loginfo("DVL Hız: [{:.2f}, {:.2f}, {:.2f}]".format(*self.dvl_velocity))
        rospy.loginfo("Engel Mesafeleri: Ön:{:.1f}m".format(self.obstacle_distances['front']))
        rospy.loginfo("Waypoint: {}/{}".format(self.current_waypoint_idx, len(self.waypoints)))

if __name__ == '__main__':
    try:
        controller = GazeboAutonomousController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Otonom kontrol sistemi durduruldu.")

