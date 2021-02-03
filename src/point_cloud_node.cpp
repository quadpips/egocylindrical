//
// Created by root on 2/5/18.
//

#include <np_egocylinder/point_cloud_generator.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_pointcloud_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    np_egocylinder::EgoCylinderPointCloudGenerator s(nh, pnh);
    s.init();
    ros::spin();
}
