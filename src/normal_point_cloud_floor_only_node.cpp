//
// Created by root on 2/5/18.
//

#include <egocylindrical/normal_point_cloud_floor_only_generator.h>
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_normal_pointcloud_floor_only_publisher");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::NormalPointCloudFloorOnlyGenerator s(nh, pnh);
    s.init();
    ros::spin();
}
