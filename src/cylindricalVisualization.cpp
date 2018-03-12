//
// Created by root on 2/5/18.
//

#include "cylindricalVisualization.h"
CylindricalVisualization::CylindricalVisualization() :it_(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
    message_filters::Subscriber<sensor_msgs::Image> depthSub(nh_, "/camera/depth/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/camera/depth/camera_info", 10);
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> timeSynchronizer(depthSub, depthInfoSub, 30);
    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::cameraCb, this, _1, _2));
    pub = it_.advertise("projected_image", 20);
    pointCloud_Pub = nh_.advertise<sensor_msgs::PointCloud2>("projected_pointCloud", 20);
    ros::spin();
}

void CylindricalVisualization::cameraCb(const sensor_msgs::ImageConstPtr &image,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_INFO("Received images and camera info");
    EgoCylindrical translated = EgoCylindrical(*image, *cam_info);

//    sensor_msgs::PointCloud2 pointCloud2;
//    pcl::toROSMsg(translated.pcloud, pointCloud2);
//    pointCloud2.header.frame_id = image->header.frame_id;
//    pointCloud_Pub.publish(pointCloud2);

    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time(0);
    msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC1, translated.toImage()).toImageMsg();
    pub.publish(msg);

}







int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_model");
    CylindricalVisualization s;
    ros::spin();
}