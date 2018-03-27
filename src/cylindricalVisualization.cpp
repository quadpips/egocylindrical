//
// Created by root on 2/5/18.
//

#include <egocylindrical/cylindricalVisualization.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/message_filter.h"

CylindricalVisualization::CylindricalVisualization() :it_(nh_), propagator_(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
//    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh_, "/camera/depth/image_raw", 2);
//    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/camera/depth/camera_info", 2);
    message_filters::Subscriber<stixel_estimator::stixelListMsg> stixelSub(nh_, "/stixels", 30);
    message_filters::Subscriber<sensor_msgs::CameraInfo> stixelInfoSub(nh_, "/multisense_sl/camera/left/camera_info", 30);

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_(buffer_);
    
    tf2_ros::MessageFilter<sensor_msgs::CameraInfo> info_tf_filter(stixelInfoSub, buffer_, "odom", 30,nh_);
    message_filters::TimeSynchronizer<stixel_estimator::stixelListMsg, sensor_msgs::CameraInfo> timeSynchronizer(stixelSub, info_tf_filter, 30);

    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::stixelCb, this, _1, _2));
//    pub = it_.advertise("projected_image", 20);
    ptPub = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical", 100);
    ptPub2 = nh_.advertise<sensor_msgs::PointCloud2>("cylindrical_original", 100);
    ros::spin();
}

void CylindricalVisualization::stixelCb(const stixel_estimator::stixelListMsgConstPtr &stixels,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_DEBUG("Received images and camera info");

    propagator_.update(stixels, cam_info);

    ROS_DEBUG("publish egocylindrical image");
    
    ptPub.publish(propagator_.getPropagatedPointCloud());
    
//    pub.publish(propagator_.getRawRangeImage());
    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_model");
    CylindricalVisualization s;
    ros::spin();
}
