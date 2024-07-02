//
// Created by root on 2/5/18.
//

#include <egocylindrical/labeled_point_cloud_generator.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_labeled_pointcloud_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::LabeledPointCloudGenerator s(nh, pnh);
    s.init();
    ros::spin();
}
