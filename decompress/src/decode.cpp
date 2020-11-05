#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

sensor_msgs::Image pub_msg_color;
sensor_msgs::Image pub_msg_depth;
sensor_msgs::CameraInfo pub_msg_info;
ros::Publisher pub_color;
ros::Publisher pub_depth;
ros::Publisher pub_info;

void colorCallback(const sensor_msgs::Image& msg)
{
    pub_msg_color = msg;
}

void depthCallback(const sensor_msgs::Image& msg)
{
    pub_msg_depth = msg;
}

void infoCallback(const sensor_msgs::CameraInfo& msg)
{
    pub_msg_info = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decode");
    ros::NodeHandle nh;
    ros::Subscriber sub_decode_color = nh.subscribe("/camera/color/image_raw/decompressed", 100, colorCallback);
    ros::Subscriber sub_decode_depth = nh.subscribe("/camera/aligned_depth_to_color/image_raw/decompressed", 100, depthCallback);
    ros::Subscriber sub_info = nh.subscribe("/camera/color/camera_info", 100, infoCallback);
    pub_color = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw/republish", 10);
    pub_depth = nh.advertise<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw/republish", 10);
    pub_info = nh.advertise<sensor_msgs::CameraInfo>("/camera/color/camera_info/republish", 10);
    
    // ros::Rate rate(100);

    while (nh.ok()) {
        pub_msg_color.header.stamp = ros::Time::now();
        pub_msg_depth.header.stamp = ros::Time::now();
        pub_msg_info.header.stamp = ros::Time::now();
        pub_color.publish(pub_msg_color);
        pub_depth.publish(pub_msg_depth);
        pub_info.publish(pub_msg_info);
        ros::spinOnce();
        // rate.sleep();
    }
}