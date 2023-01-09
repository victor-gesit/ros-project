#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3, Point
from sensor_msgs.msg import Range, LaserScan, Image
from nav_msgs.msg import Odometry
import math
import cv2
from cv_bridge import CvBridge, CvBridgeError

class VFI20Rosbot:
    def __init__(self):
        self.vel_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        # Range Subsribers
        rospy.Subscriber("range/fl", Range, callback=self.front_left_range_callback)
        rospy.Subscriber("range/fr", Range, callback=self.front_right_range_callback)
        rospy.Subscriber("range/rl", Range, callback=self.rear_left_range_callback)
        rospy.Subscriber("range/rr", Range, callback=self.rear_right_range_callback)
        
        # LiDar Subscriber
        rospy.Subscriber("scan", LaserScan, callback=self.lidar_callback)

        # RPY Subscriber
        rospy.Subscriber("rpy", Vector3, callback=self.rpy_callback)
        
        # Odometer Subscriber
        rospy.Subscriber("odom", Odometry, callback=self.odometry_callback)

        # Camera Subscriber
        rospy.Subscriber("camera/rgb/image_raw", Image, callback=self.camera_callback)
        self.bridge = CvBridge()
        


        self.front_left_range = Range()
        self.front_right_range = Range()
        self.rear_left_range = Range()
        self.rear_right_range = Range()

        self.laser_range = LaserScan()

        self.rounding_obstacle = False
        self.finding_course = False

        self.left_turn_recommended_radar = False
        self.right_turn_recommended_radar = False

        self.left_turn_recommended_lidar = False
        self.right_turn_recommended_lidar = False

        self.minimum_front_clearance = 0.3
        self.minimum_front_clearance_to_begin_turn = 0.8
        self.maximum_proximity = 1.5


        self.max_x_velocity = 0.5
        self.current_x_velocity = 0
        self.current_z_velocity = 0
        self.current_z_orientation = 0
        self.radar_range_to_consider = 0.8

        # LIDAR
        # Front
        self.front_obstacle_distance_lidar = 0
        self.front_right_obstacle_distance_lidar = 0
        self.front_left_obstacle_distance_lidar = 0
        self.lidar_ranges_0_to_180 = [] # Front Left
        self.lidar_ranges_540_to_719 = [] # Front Right
        # Sides
        self.side_minimum_clearance = 0.1
        self.left_side_obstacle_distance_lidar = 0
        self.right_side_obstacle_distance_lidar = 0

        #RADAR
        # Front
        self.front_right_obstacle_distance_radar = 0
        self.front_left_obstacle_distance_radar = 0
        
        self.obstacle_on_left = False
        self.obstacle_on_right = False
        initial_vel = Twist()
        initial_vel.linear.x = 0.1
        initial_vel.angular.z = 0.1
        self.rosbot_vel = initial_vel

        #ODOMETER
        self.odometer_position = Point()
        self.desired_y_line = 1.0
        self.initial_position_measured = False
        self.total_distance_travelled = 0
        self.previous_x_coordinate = 0
        self.previous_y_coordinate = 0
        self.task_completed = False

        #CAMERA
        self.previous_frame = []
        self.previous_frame_initialized = False
        self.left_turn_recommended_camera = False
        self.right_turn_recommended_camera = False

        self.rosbot_state = 0 # 0: Idle, 1: Moving forward, 2: Moving forward and reorienting, 3: Reorienting, 4: Avoiding obstacle
        pass

    def start_robot(self):
        self.rosbot_state = 2
        pass

    def rpy_callback(self, rpy):
        self.current_z_orientation = rpy.z

    def odometry_callback(self, odometry):
        self.odometer_position = odometry.pose.pose.position
        
        # Distance Travelled
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        if not self.initial_position_measured:
            self.previous_x_coordinate = x
            self.previous_y_coordinate = y
            self.initial_position_measured = True

        distance_travelled = math.sqrt(pow((x-self.previous_x_coordinate), 2) + pow((y-self.previous_y_coordinate), 2))
        self.total_distance_travelled += distance_travelled
        self.previous_x_coordinate = x
        self.previous_y_coordinate = y

        if x <= -4:
            self.task_completed = True

        # print("Distance Travelled: ", self.total_distance_travelled, x)

    def camera_callback(self, image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)


        cv_image_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, threshold = cv2.threshold(cv_image_gray, 127, 255, cv2.THRESH_BINARY_INV)
        cv_image_canny = cv2.Canny(cv_image_gray, 30, 200)

        contours, _ = cv2.findContours(threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_to_draw = []
        for contour in contours:
            if cv2.contourArea(contour) > 800:
                contours_to_draw.append(contour)
            # x, y, w, h = cv2.boundingRect(contour)
            # cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 0, 255), 2)


        cv2.drawContours(cv_image, contours_to_draw, -1, (255, 0, 0), 3)
        extreme_bottom_coordinates = []
        closest_to_car_obstacle_coordinates = (0, 0)
        for contour in contours_to_draw:
            ext_bottom = tuple(contour[contour[:, :, 1].argmax()][0])
            extreme_bottom_coordinates.append(ext_bottom)
            if ext_bottom[0] > closest_to_car_obstacle_coordinates[0]:
                closest_to_car_obstacle_coordinates = ext_bottom
        
        (height, width, _) = cv_image.shape
        image_height_to_begin_turning = 0.78
        image_height_to_stop = 0.95
        index_of_bottom = int(height * image_height_to_begin_turning)
        cv2.line(cv_image, (0, index_of_bottom), (width, index_of_bottom), (0, 255, 0), 2)
        
        if closest_to_car_obstacle_coordinates[1] >= index_of_bottom:
            mid_line = int(width/2)
            if closest_to_car_obstacle_coordinates[0] > mid_line:
                self.left_turn_recommended_camera = True
                self.right_turn_recommended_camera = False
                print("Turn Left Camera")
            else:
                self.left_turn_recommended_camera = False
                self.right_turn_recommended_camera = True
                print("Turn Right Camera")
        else:
            self.left_turn_recommended_camera = False
            self.right_turn_recommended_camera = False
        
        cv2.imshow("Video Stream: ", cv_image)
        cv2.imshow("Canny Stream: ", threshold)
        cv2.waitKey(3)
        pass

    def re_orient_rosbot(self):
        if self.task_completed:
            self.stop_car()
            return
        if self.rosbot_state == 3:
            vel = Twist()
            if self.right_turn_recommended_radar:
                vel.angular.z = -1
            elif self.left_turn_recommended_radar:
                vel.angular.z = 1
            elif self.right_turn_recommended_lidar:
                vel.angular.z = -1
            elif self.left_turn_recommended_lidar:
                vel.angular.z = 1
            elif self.right_turn_recommended_camera:
                print("Turning by Camera RIGHT")
                vel.linear.x = 0.1
                vel.angular.z = -1
            elif self.left_turn_recommended_camera:
                print("Turning by camera LEFT")
                vel.linear.x = 0.1
                vel.angular.z = 1
            else:
                vel.angular.z = self.current_z_velocity
            vel.linear.x = self.current_x_velocity
            self.vel_publisher.publish(vel)
                
        else:
            z = self.current_z_orientation
            acceptable_displacement_tolerance = 0.1
            max_adjustment_angle = 45
            adjustment_angle = 0


            displacement_from_path = self.odometer_position.y - self.desired_y_line
            if abs(displacement_from_path) > acceptable_displacement_tolerance:
                adjustment_angle = displacement_from_path * max_adjustment_angle
                adjustment_angle = adjustment_angle/abs(adjustment_angle) * min([abs(adjustment_angle), max_adjustment_angle])
                pass
            z_vel = 0


            if adjustment_angle > 0: # Off to the right
                if z < 0: # Car facing left
                    z = z - abs(adjustment_angle) if z - abs(adjustment_angle) >= -179 else -179 - (z - abs(adjustment_angle))
                if z > 0: # Car facing right
                    z = z - abs(adjustment_angle) if z - abs(adjustment_angle) > 0 else 0
            if adjustment_angle < 0: # Off to the left
                if z < 0: # Car facing left
                    z = z + abs(adjustment_angle) if z + abs(adjustment_angle) < 0 else 0
                if z > 0: # Car facing right
                    z = z + abs(adjustment_angle) if z + abs(adjustment_angle) < 180 else -359 + z + abs(adjustment_angle)
                
            z = z/abs(z) * min([abs(z), 179]) if z != 0 else 0


            if abs(z) < 90:
                z_vel = 1
            else:
                z_vel = (-1/90) * abs(z) + 2
            
                
            z_vel = z/abs(z) * z_vel if z != 0 else 0
            self.current_z_velocity = z_vel
            vel = Twist()
            vel.angular.z = z_vel
            vel.linear.x = self.current_x_velocity
            self.vel_publisher.publish(vel)
            pass

    def lidar_callback(self, laser_scan):
        front_points_angle = 70
        front = laser_scan.ranges[0]
        self.lidar_ranges_0_to_180 = laser_scan.ranges[0:180]
        self.lidar_ranges_540_to_719 = laser_scan.ranges[540:719]
        front_left = min(laser_scan.ranges[0:front_points_angle])
        front_right = min(laser_scan.ranges[(719 - front_points_angle): 719])
        right = laser_scan.ranges[540]

        self.front_obstacle_distance_lidar = front
        self.front_right_obstacle_distance_lidar = front_right
        self.front_left_obstacle_distance_lidar = front_left

        self.laser_range = laser_scan
        
        self.chose_direction_from_lidar_data()
        self.combine_sensors()
        self.move()
    
    def chose_direction_from_lidar_data(self):
        left_range = self.lidar_ranges_0_to_180
        right_range = self.lidar_ranges_540_to_719[::-1]
        minimum_contiguous_points_count = 10

        left_contiguous_points_start_index = 0
        right_contiguous_points_start_index = 0

        furthest_point_on_left = 0
        furthest_point_on_right = 0

        contiguous_points_found_on_left = False
        contiguous_points_found_on_right = False

        index_of_infinity_left = 180
        index_of_infinity_right = 180

        while not contiguous_points_found_on_left and left_contiguous_points_start_index < len(left_range):
            point_distance = left_range[left_contiguous_points_start_index]
            if point_distance > self.minimum_front_clearance_to_begin_turn:
                stop_index = min([len(left_range), left_contiguous_points_start_index + minimum_contiguous_points_count])
                
                if float('inf') in left_range:
                    index_of_infinity_left = left_range.index(float('inf'))
                closest = min(left_range[left_contiguous_points_start_index:stop_index])
                
                if closest >= self.minimum_front_clearance:
                    contiguous_points_found_on_left = True
                else:
                    left_contiguous_points_start_index += 1
            else:
                left_contiguous_points_start_index += 1

        while not contiguous_points_found_on_right and right_contiguous_points_start_index < len(right_range):
            point_distance = right_range[right_contiguous_points_start_index]
            if point_distance > self.minimum_front_clearance_to_begin_turn:
                stop_index = min([len(right_range), right_contiguous_points_start_index + minimum_contiguous_points_count])
                if float('inf') in right_range:
                    index_of_infinity_right = right_range.index(float('inf'))
                closest = min(right_range[right_contiguous_points_start_index:stop_index])
                if closest >= self.minimum_front_clearance:
                    contiguous_points_found_on_right = True
                else:
                    right_contiguous_points_start_index += 1
            else:
                right_contiguous_points_start_index += 1

        
        go_left_by_contiguous_points = contiguous_points_found_on_left and left_contiguous_points_start_index < right_contiguous_points_start_index
        go_right_by_contiguous_points = contiguous_points_found_on_right and right_contiguous_points_start_index < left_contiguous_points_start_index
        

        go_left_by_infinity_value = index_of_infinity_left != 180 and index_of_infinity_left < index_of_infinity_right
        go_right_by_infinity_value = index_of_infinity_right != 180 and index_of_infinity_right <= index_of_infinity_left


        closest_left = self.front_left_obstacle_distance_lidar
        closest_right = self.front_right_obstacle_distance_lidar

        if left_contiguous_points_start_index == right_contiguous_points_start_index:
            if closest_left <= self.minimum_front_clearance_to_begin_turn or closest_right <= self.minimum_front_clearance_to_begin_turn:
                if closest_left < closest_right:
                    self.left_turn_recommended_lidar = False
                    self.right_turn_recommended_lidar = True
                else:
                    self.left_turn_recommended_lidar = True
                    self.right_turn_recommended_lidar = False
            else:
                self.left_turn_recommended_lidar = False
                self.right_turn_recommended_lidar = False
        elif go_left_by_contiguous_points:
            self.left_turn_recommended_lidar = True
            self.right_turn_recommended_lidar = False
        elif go_right_by_contiguous_points:
            self.right_turn_recommended_lidar = True
            self.left_turn_recommended_lidar = False

    def combine_sensors(self):
        radar_fl = self.front_left_obstacle_distance_radar
        radar_fr = self.front_right_obstacle_distance_radar
        lidar_fl = self.front_left_obstacle_distance_lidar
        lidar_fr = self.front_right_obstacle_distance_lidar
        lidar_left = self.left_side_obstacle_distance_lidar
        lidar_right = self.right_side_obstacle_distance_lidar
        min_front_clearance = self.minimum_front_clearance
        min_front_clearance_to_begin_turn = self.minimum_front_clearance_to_begin_turn

        # OBSTACLE AVOIDANCE
        closest_fl = min([radar_fl, lidar_fl])
        closest_fr= min([radar_fr, lidar_fr])
        obstacle_on_left = lidar_left <= self.side_minimum_clearance
        obstacle_on_right = lidar_right <= self.side_minimum_clearance
        begin_obstacle_avoidance = closest_fl < min_front_clearance_to_begin_turn or closest_fr < min_front_clearance_to_begin_turn
        camera_detects_obstacle = self.left_turn_recommended_camera or self.right_turn_recommended_camera
        lidar_detects_obstacle = self.left_turn_recommended_lidar or self.right_turn_recommended_lidar
        
        if begin_obstacle_avoidance or camera_detects_obstacle or lidar_detects_obstacle:
            self.rosbot_state = 3
            if radar_fl > radar_fr:
                self.left_turn_recommended_radar = True
                self.right_turn_recommended_radar = False
            elif radar_fl < radar_fr:
                self.right_turn_recommended_radar = True
                self.left_turn_recommended_radar = False
            else:
                self.left_turn_recommended_radar = False
                self.right_turn_recommended_radar = False
        else:
            self.rosbot_state = 2
    
    def move(self):
        self.move_forward()
        self.re_orient_rosbot()


    def move_forward(self):
        if self.task_completed:
            self.stop_car()
            return

        all_sensor_distances = [self.front_obstacle_distance_lidar, self.front_right_obstacle_distance_lidar, self.front_left_obstacle_distance_lidar, self.front_left_obstacle_distance_radar, self.front_right_obstacle_distance_radar]
        
        smallest_distance_in_front = min(all_sensor_distances)
        velocity_x = 0
        if smallest_distance_in_front <= self.minimum_front_clearance:
            velocity_x = 0
        elif smallest_distance_in_front > self.maximum_proximity:
            velocity_x = self.max_x_velocity
        else:
            velocity_x = (0.5/1.15) * smallest_distance_in_front - 0.1521 + 0.1
        
        vel = Twist()
        vel.linear.x = velocity_x
        self.current_x_velocity = velocity_x
        vel.angular.z = self.current_z_velocity
        self.vel_publisher.publish(vel)
        pass

    def stop_car(self):
        self.vel_publisher.publish(Twist())

    def front_left_range_callback(self, range):
        self.front_left_range = range
        self.front_left_obstacle_distance_radar = float("inf") if range.range > self.radar_range_to_consider else range.range
        
        if self.front_left_obstacle_distance_radar < 0.4 and self.front_right_obstacle_distance_radar > self.front_left_obstacle_distance_radar:
            self.right_turn_recommended_radar = True
        pass

    def front_right_range_callback(self, range):
        self.front_right_range = range
        self.front_right_obstacle_distance_radar = float("inf") if range.range > self.radar_range_to_consider else range.range

        if self.front_right_obstacle_distance_radar < 0.4 and self.front_left_obstacle_distance_radar > self.front_left_obstacle_distance_radar:
            self.left_turn_recommended_radar = True
        pass

    def rear_left_range_callback(self, range):
        self.front_right_range = float("inf") if range.range > self.radar_range_to_consider else range.range
        pass

    def rear_right_range_callback(self, range):
        self.rear_right_range = float("inf") if range.range > self.radar_range_to_consider else range.range
        pass

if __name__ == "__main__":
    rospy.init_node("victor_rosbot_main_node")
    vfi20rosbot = VFI20Rosbot()
    rospy.sleep(5)
    vfi20rosbot.start_robot()
    rospy.spin()
