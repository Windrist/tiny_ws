#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from filterpy.kalman import ExtendedKalmanFilter
# from filterpy.common import Q_continuous_white_noise, Q_discrete_white_noise

class Calculate_Cov():

    def __init__(self):
        
        self.ctrl_c = False
        self.PI = 3.141592654
        self.num = np.zeros(150000000).reshape(15, -1)
        self.ave = np.zeros(15)
        self.all_sum = np.zeros(15)
        self.last = np.zeros(15)
        self.sum = np.zeros(15)
        self.n = np.ones(15)
        self.my_yaw = 0
        self.theta = 0
        self.prev_yaw = 0
        self.init_roll = 0
        self.init_pitch = 0
        self.init_yaw = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.fuso_odom = Odometry()
        self.fuso_imu = Imu()
        self.quat_msg = Quaternion()
        self.fuso_pose = Odometry()
        self.odom_publisher = rospy.Publisher('/fuso/odom', Odometry, queue_size=1)
        # self.imu_publisher = rospy.Publisher('/fuso/imu', Imu, queue_size=10)
        # self.pose_publisher = rospy.Publisher('/fuso/pose', PoseWithCovarianceStamped, queue_size=10)
        self.pose_publisher = rospy.Publisher('/fuso/pose', Odometry, queue_size=1)
        self.odom_subscriber = rospy.Subscriber('/odom2', Odometry, self.odom_callback)
        # self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.pose_subscriber = rospy.Subscriber('/odom1', Odometry, self.pose_callback)
        # self.initial_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_callback)
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(100)

    def shutdownhook(self):
        # Works better than the rospy.is_shut_down()
        self.ctrl_c = True
    
    def covariancePub(self, x, i):
        self.num[i, int(self.n[i])] = x
        self.sum[i] = self.sum[i] + x
        self.ave[i] = self.sum[i] / self.n[i]
        self.all_sum[i] = 0
        for j in range(int(self.n[i])):
            self.all_sum[i] = self.all_sum[i] + (self.num[i][j] - self.ave[i]) * (self.num[i][j] - self.ave[i])
        self.n[i] = self.n[i] + 1
        return self.all_sum[i] / (self.n[i] - 1)

    def odom_callback(self, msg):
        # Get Odom Message
        self.fuso_odom = msg
        self.fuso_odom.header.stamp = rospy.Time.now()
        self.fuso_odom.child_frame_id = "odom"
        self.fuso_odom.header.frame_id = "base_link"
        
        # Calculate and Set Covariance
        self.fuso_odom.twist.covariance = list(self.fuso_odom.twist.covariance)
        self.fuso_odom.twist.covariance[0] = self.covariancePub(self.fuso_odom.twist.twist.linear.x, 6)
        self.fuso_odom.twist.covariance[7] = self.covariancePub(self.fuso_odom.twist.twist.linear.y, 7)
        self.fuso_odom.twist.covariance[14] = self.covariancePub(self.fuso_odom.twist.twist.linear.z, 8)
        
        if self.fuso_odom.twist.covariance[0] == 0 and self.fuso_odom.twist.twist.linear.x == 0:
            self.fuso_odom.twist.covariance[0] = 1e-9
        elif self.fuso_odom.twist.covariance[0] == 0 and self.fuso_odom.twist.twist.linear.x != 0: 
            self.fuso_odom.twist.covariance[0] = self.last[6]
        if self.fuso_odom.twist.covariance[7] == 0 and self.fuso_odom.twist.twist.linear.y == 0:
            self.fuso_odom.twist.covariance[7] = 1e-9
        elif self.fuso_odom.twist.covariance[7] == 0 and self.fuso_odom.twist.twist.linear.y != 0: 
            self.fuso_odom.twist.covariance[7] = self.last[7]
        if self.fuso_odom.twist.covariance[14] == 0 and self.fuso_odom.twist.twist.linear.z == 0:
            self.fuso_odom.twist.covariance[14] = 1e-9
        elif self.fuso_odom.twist.covariance[14] == 0 and self.fuso_odom.twist.twist.linear.z != 0: 
            self.fuso_odom.twist.covariance[14] = self.last[8]
            
        self.last[6] = self.fuso_odom.twist.covariance[0]
        self.last[7] = self.fuso_odom.twist.covariance[7]
        self.last[8] = self.fuso_odom.twist.covariance[14]
        
        self.fuso_odom.twist.covariance = tuple(self.fuso_odom.twist.covariance)
        
        # Publish Odom
        self.odom_publisher.publish(self.fuso_odom)
        
    # def imu_callback(self, msg):
    #     # Get Imu Message
    #     self.fuso_imu = msg
    #     # self.fuso_imu.orientation.x = msg.orientation.x
    #     # self.fuso_imu.orientation.y = msg.orientation.y
    #     # self.fuso_imu.orientation.z = msg.orientation.z
    #     # self.fuso_imu.orientation.w = msg.orientation.w
        
    #     # Calculate and Set Covariance
    #     (self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.fuso_imu.orientation.x, self.fuso_imu.orientation.y, self.fuso_imu.orientation.z, self.fuso_imu.orientation.w])
    #     # self.my_yaw = self.yaw - self.prev_yaw
    #     # if self.my_yaw < - PI:
	#     # self.my_yaw += 2 * PI
    #     # elif self.my_yaw > PI:
	#     # self.my_yaw -= 2 * PI
    #     # self.my_yaw += self.init_yaw
    #     # self.quat_msg = quaternion_from_euler(self.roll, self.pitch, self.my_yaw)
    #     # self.fuso_imu.orientation = self.quat_msg

    #     self.fuso_imu.orientation_covariance = list(self.fuso_imu.orientation_covariance)
    #     self.fuso_imu.angular_velocity_covariance = list(self.fuso_imu.angular_velocity_covariance)
    #     self.fuso_imu.linear_acceleration_covariance = list(self.fuso_imu.linear_acceleration_covariance)
    #     self.fuso_imu.orientation_covariance[0] = self.covariancePub(self.roll, 3)
    #     self.fuso_imu.orientation_covariance[4] = self.covariancePub(self.pitch, 4)
    #     # self.fuso_imu.orientation_covariance[8] = self.covariancePub(self.my_yaw, 5)
    #     self.fuso_imu.orientation_covariance[8] = self.covariancePub(self.yaw, 5)
    #     self.fuso_imu.angular_velocity_covariance[0] = self.covariancePub(self.fuso_imu.angular_velocity.x, 9)
    #     self.fuso_imu.angular_velocity_covariance[4] = self.covariancePub(self.fuso_imu.angular_velocity.y, 10)
    #     self.fuso_imu.angular_velocity_covariance[8] = self.covariancePub(self.fuso_imu.angular_velocity.z, 11)
    #     self.fuso_imu.linear_acceleration_covariance[0] = self.covariancePub(self.fuso_imu.linear_acceleration.x, 12)
    #     self.fuso_imu.linear_acceleration_covariance[4] = self.covariancePub(self.fuso_imu.linear_acceleration.y, 13)
    #     self.fuso_imu.linear_acceleration_covariance[8] = self.covariancePub(self.fuso_imu.linear_acceleration.z, 14)
        
    #     if self.fuso_imu.orientation_covariance[0] == 0 and self.roll == 0:
    #         self.fuso_imu.orientation_covariance[0] = 1e-9
    #     elif self.fuso_imu.orientation_covariance[0] == 0 and self.roll != 0:
    #         self.fuso_imu.orientation_covariance[0] = self.last[3]
    #     if self.fuso_imu.orientation_covariance[4] == 0 and self.pitch == 0:
    #         self.fuso_imu.orientation_covariance[4] = 1e-9
    #     elif self.fuso_imu.orientation_covariance[4] == 0 and self.pitch != 0:
    #         self.fuso_imu.orientation_covariance[4] = self.last[4]      
    #     # if self.fuso_imu.orientation_covariance[8] == 0 and self.my_yaw == 0:
    #     if self.fuso_imu.orientation_covariance[8] == 0 and self.yaw == 0:
    #         self.fuso_imu.orientation_covariance[8] = 1e-9
    #     # elif self.fuso_imu.orientation_covariance[8] == 0 and self.my_yaw != 0:
    #     elif self.fuso_imu.orientation_covariance[8] == 0 and self.yaw != 0:
    #         self.fuso_imu.orientation_covariance[8] = self.last[5]
    #     if self.fuso_imu.angular_velocity_covariance[0] == 0 and self.fuso_imu.angular_velocity.x == 0:
    #         self.fuso_imu.angular_velocity_covariance[0] = 1e-9
    #     elif self.fuso_imu.angular_velocity_covariance[0] == 0 and self.fuso_imu.angular_velocity.x != 0:
    #         self.fuso_imu.angular_velocity_covariance[0] = self.last[9]
    #     if self.fuso_imu.angular_velocity_covariance[4] == 0 and self.fuso_imu.angular_velocity.y == 0:
    #         self.fuso_imu.angular_velocity_covariance[4] == 1e-9
    #     elif self.fuso_imu.angular_velocity_covariance[4] == 0 and self.fuso_imu.angular_velocity.y != 0:
    #         self.fuso_imu.angular_velocity_covariance[4] = self.last[10]
    #     if self.fuso_imu.angular_velocity_covariance[8] == 0 and self.fuso_imu.angular_velocity.z == 0:
    #         self.fuso_imu.angular_velocity_covariance[8] == 1e-9
    #     elif self.fuso_imu.angular_velocity_covariance[8] == 0 and self.fuso_imu.angular_velocity.z != 0:
    #         self.fuso_imu.angular_velocity_covariance[8] = self.last[11]
    #     if self.fuso_imu.linear_acceleration_covariance[0] == 0 and self.fuso_imu.linear_acceleration.x == 0:
    #         self.fuso_imu.linear_acceleration_covariance[0] == 1e-9
    #     elif self.fuso_imu.linear_acceleration_covariance[0] == 0 and self.fuso_imu.linear_acceleration.x != 0:
    #         self.fuso_imu.linear_acceleration_covariance[0] = self.last[12]
    #     if self.fuso_imu.linear_acceleration_covariance[4] == 0 and self.fuso_imu.linear_acceleration.y == 0:
    #         self.fuso_imu.linear_acceleration_covariance[4] == 1e-9
    #     elif self.fuso_imu.linear_acceleration_covariance[4] == 0 and self.fuso_imu.linear_acceleration.y != 0:
    #         self.fuso_imu.linear_acceleration_covariance[4] = self.last[13]
    #     if self.fuso_imu.linear_acceleration_covariance[8] == 0 and self.fuso_imu.linear_acceleration.z == 0:
    #         self.fuso_imu.linear_acceleration_covariance[8] == 1e-9
    #     elif self.fuso_imu.linear_acceleration_covariance[8] == 0 and self.fuso_imu.linear_acceleration.z != 0:
    #         self.fuso_imu.linear_acceleration_covariance[8] = self.last[14]
        
    #     self.last[3] = self.fuso_imu.orientation_covariance[0]
    #     self.last[4] = self.fuso_imu.orientation_covariance[4]
    #     self.last[5] = self.fuso_imu.orientation_covariance[8]
    #     self.last[9] = self.fuso_imu.angular_velocity_covariance[0]
    #     self.last[10] = self.fuso_imu.angular_velocity_covariance[4]
    #     self.last[11] = self.fuso_imu.angular_velocity_covariance[8]
    #     self.last[12] = self.fuso_imu.linear_acceleration_covariance[0]
    #     self.last[13] = self.fuso_imu.linear_acceleration_covariance[4]
    #     self.last[14] = self.fuso_imu.linear_acceleration_covariance[8]
        
    #     self.fuso_imu.orientation_covariance = tuple(self.fuso_imu.orientation_covariance)
    #     self.fuso_imu.angular_velocity_covariance = tuple(self.fuso_imu.angular_velocity_covariance)
    #     self.fuso_imu.linear_acceleration_covariance = tuple(self.fuso_imu.linear_acceleration_covariance)

    #     # Publish Imu
    #     self.imu_publisher.publish(self.fuso_imu)
         
    def pose_callback(self, msg):
        # Get Pose Message
        self.fuso_pose = msg
        
        # Calculate and Set Covariance
        self.fuso_pose.pose.covariance = list(self.fuso_pose.pose.covariance)
        self.fuso_pose.pose.covariance[0] = self.covariancePub(self.fuso_pose.pose.pose.position.x, 0)
        self.fuso_pose.pose.covariance[7] = self.covariancePub(self.fuso_pose.pose.pose.position.y, 1)
        self.fuso_pose.pose.covariance[14] = self.covariancePub(self.fuso_pose.pose.pose.position.z, 2)
        
        # (self.roll, self.pitch, self.yaw) = euler_from_quaternion([self.fuso_pose.pose.pose.orientation.x, self.fuso_pose.pose.pose.orientation.y, self.fuso_pose.pose.pose.orientation.z, self.fuso_pose.pose.pose.orientation.w])
        # self.fuso_pose.pose.covariance[21] = self.covariancePub(self.roll, 3)
        # self.fuso_pose.pose.covariance[28] = self.covariancePub(self.pitch, 4)
        # self.fuso_pose.pose.covariance[35] = self.covariancePub(self.yaw, 5)
        if self.fuso_pose.pose.covariance[0] == 0 and self.fuso_pose.pose.pose.position.x == 0:
            self.fuso_pose.pose.covariance[0] = 1e-9
        elif self.fuso_pose.pose.covariance[0] == 0 and self.fuso_pose.pose.pose.position.x != 0:
            self.fuso_pose.pose.covariance[0] = self.last[0]
        if self.fuso_pose.pose.covariance[7] == 0 and self.fuso_pose.pose.pose.position.y == 0:
            self.fuso_pose.pose.covariance[7] = 1e-9
        elif self.fuso_pose.pose.covariance[7] == 0 and self.fuso_pose.pose.pose.position.y != 0:
            self.fuso_pose.pose.covariance[7] = self.last[1]
        if self.fuso_pose.pose.covariance[14] == 0 and self.fuso_pose.pose.pose.position.z == 0:
            self.fuso_pose.pose.covariance[14] = 1e-9
        elif self.fuso_pose.pose.covariance[14] == 0 and self.fuso_pose.pose.pose.position.z != 0:
            self.fuso_pose.pose.covariance[14] = self.last[2]

        # if self.fuso_pose.pose.covariance[21] == 0 and self.roll == 0:
        #     self.fuso_pose.pose.covariance[21] == 1e-9
        # elif self.fuso_pose.pose.covariance[21] == 0 and self.roll != 0:
        #     self.fuso_pose.pose.covariance[21] = self.last[3]
        # if self.fuso_pose.pose.covariance[28] == 0 and self.pitch == 0:
        #     self.fuso_pose.pose.covariance[28] == 1e-9
        # elif self.fuso_pose.pose.covariance[28] == 0 and self.pitch != 0:
        #     self.fuso_pose.pose.covariance[28] = self.last[4]
        # if self.fuso_pose.pose.covariance[35] == 0 and self.yaw == 0:
        #     self.fuso_pose.pose.covariance[35] == 1e-9
        # elif self.fuso_pose.pose.covariance[35] == 0 and self.yaw != 0:
        #     self.fuso_pose.pose.covariance[35] = self.last[5]
        
        self.last[0] = self.fuso_pose.pose.covariance[0]
        self.last[1] = self.fuso_pose.pose.covariance[7]
        self.last[2] = self.fuso_pose.pose.covariance[14]
        # self.last[3] = self.fuso_pose.pose.covariance[21]
        # self.last[4] = self.fuso_pose.pose.covariance[28]
        # self.last[5] = self.fuso_pose.pose.covariance[35]
        
        self.fuso_pose.pose.covariance = tuple(self.fuso_pose.pose.covariance)
        
        # Publish Pose
        self.pose_publisher.publish(self.fuso_pose)
    
    # def initial_callback(self, msg):
    #     # Get Initial Pose Message
    #     (self.init_roll, self.init_pitch, self.init_yaw) = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    #     self.prev_yaw = self.yaw
   
    def publish(self):
        while not rospy.is_shutdown():
            # self.odom_publisher.publish(self.fuso_odom)
            # self.imu_publisher.publish(self.fuso_imu)
            # self.pose_publisher.publish(self.fuso_pose)
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('calculate_covariance', anonymous=True)
    Cov_object = Calculate_Cov()
    try:
        Cov_object.publish()
    except rospy.ROSInterruptException:
        pass
