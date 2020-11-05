#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from filterpy.kalman import ExtendedKalmanFilter
from filterpy.common import Q_continuous_white_noise, Q_discrete_white_noise

class Calculate_Q():
    
    def __init__(self):
        self.odom_subscriber = rospy.Subscriber('/fuso/odom', Odometry, self.odom_callback)
        self.pose_subscriber = rospy.Subscriber('/fuso/pose', Odometry, self.pose_callback)
        self.imu_subscriber = rospy.Subscriber('/fuso/imu', Imu, self.imu_callback)
        self.fuso_odom = Odometry()
        self.fuso_pose = PoseWithCovarianceStamped()
        self.fuso_imu = Imu()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(100)
    
    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.ctrl_c = True

    def odom_callback(self, msg):
        # Get Odom Message
        self.fuso_odom = msg
        
        # Calculate Process Noise
        # Qvx = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_odom.twist.covariance[0],block_size=1, order_by_dim=False)
        # Qvy = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_odom.twist.covariance[7],block_size=1, order_by_dim=False)
        # Qvz = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_odom.twist.covariance[14],block_size=1, order_by_dim=False)
        # print('Linear Velocity x/Linear Acceleration x: ', Qvx)
        # print('Linear Velocity y/Linear Acceleration y: ', Qvy)
        # print('Linear Velocity z/Linear Acceleration z: ', Qvz)
        
    def imu_callback(self, msg):
        # Get Imu Message
        self.fuso_imu = msg
        
        # Calculate Process Noise
        # Qr = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.orientation_covariance[0],block_size=1, order_by_dim=False)
        # Qp = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.orientation_covariance[4],block_size=1, order_by_dim=False)
        # Qya = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.orientation_covariance[8],block_size=1, order_by_dim=False)
        # Qar = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.angular_velocity_covariance[0],block_size=1, order_by_dim=False)
        # Qap = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.angular_velocity_covariance[4],block_size=1, order_by_dim=False)
        # Qaya = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.angular_velocity_covariance[8],block_size=1, order_by_dim=False)
        # Qax = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.linear_acceleration_covariance[0],block_size=1, order_by_dim=False)
        # Qay = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.linear_acceleration_covariance[4],block_size=1, order_by_dim=False)
        # Qaz = Q_discrete_white_noise(2,dt=0.1,var=self.fuso_imu.linear_acceleration_covariance[8],block_size=1, order_by_dim=False)
        # print('Roll/Angular Velocity x: ',Qr)
        # print('Pitch/Angular Velocity y: ', Qp)
        # print('Yaw/Angular Velocity z: ', Qya)
        # print('Angular Velocity x: ', Qar)
        # print('Angular Velocity y: ', Qap)
        # print('Angular Velocity z: ', Qaya)
        # print('Linear Acceleration x: ', Qax)
        # print('Linear Acceleration y: ', Qay)
        # print('Linear Acceleration z: ', Qaz)
        
    def pose_callback(self, msg):
        # Get Pose Message
        self.fuso_pose = msg
        # Qx = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_pose.pose.covariance[0],block_size=1, order_by_dim=False)
        # Qy = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_pose.pose.covariance[7],block_size=1, order_by_dim=False)
        # Qz = Q_discrete_white_noise(3,dt=0.1,var=self.fuso_pose.pose.covariance[14],block_size=1, order_by_dim=False)
        # print('Position x/Linear Velocity x/Linear Acceleration x: ', Qx)
        # print('Position y/Linear Velocity y/Linear Acceleration y: ', Qy)
        # print('Position z/Linear Velocity z/Linear Acceleration z: ', Qz)
        
    def publish(self):
        # loop to publish the noisy odometry values
        while not rospy.is_shutdown():
            Qp = Q_discrete_white_noise(3,dt=0.1,var=1e-2,block_size=3, order_by_dim=False)
            Qo = Q_discrete_white_noise(2,dt=0.1,var=1e-4,block_size=3, order_by_dim=False)
            print(Qp)
            print(Qo)
            self.rate.sleep()
            
if __name__ == '__main__':
    rospy.init_node('calculate_process_noise_matrix', anonymous=True)
    Q_object = Calculate_Q()
    try:
        Q_object.publish()
    except rospy.ROSInterruptException:
        pass