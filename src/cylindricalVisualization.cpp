//
// Created by root on 2/5/18.
//

#include "cylindricalVisualization.h"
CylindricalVisualization::CylindricalVisualization() :it_(nh_)
{
    std::cout<<"Visualization Node Initialized"<<std::endl;
    message_filters::Subscriber<stixel_estimator::stixelListMsg> stixelSub(nh_, "/stixels", 20);
    message_filters::Subscriber<sensor_msgs::CameraInfo> depthInfoSub(nh_, "/multisense_sl/camera/left/camera_info", 20);
    message_filters::TimeSynchronizer<stixel_estimator::stixelListMsg, sensor_msgs::CameraInfo> timeSynchronizer(stixelSub, depthInfoSub, 40);
    timeSynchronizer.registerCallback(boost::bind(&CylindricalVisualization::stixelCb, this, _1, _2));
    pub = it_.advertise("projected_image", 20);
    pointCloud_Pub = nh_.advertise<sensor_msgs::PointCloud2>("projected_pointCloud", 20);
    ros::spin();
}


void CylindricalVisualization::stixelCb(const stixel_estimator::stixelListMsgConstPtr &stixels,
                                        const sensor_msgs::CameraInfoConstPtr &cam_info)
{
    ROS_INFO("Building Egocylindrical");
    EgoCylindrical translated = EgoCylindrical(*stixels, *cam_info);

    sensor_msgs::PointCloud2 pointCloud2;
    pcl::toROSMsg(translated.pcloud, pointCloud2);
    pointCloud2.header.frame_id = cam_info->header.frame_id;
    pointCloud_Pub.publish(pointCloud2);




    cv::imshow("test", translated.toImage());
    cv::waitKey(1);
    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time(0);
    msg = cv_bridge::CvImage(header, "bgr8", translated.toImage()).toImageMsg();
    pub.publish(msg);
}







int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_model");
    CylindricalVisualization s;
    ros::spin();
}