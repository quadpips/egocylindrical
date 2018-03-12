//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_CYLINDRICALVISUALIZATION_H
#define EGOCYLINDRICAL_CYLINDRICALVISUALIZATION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include "egocylindrical.h"

class CylindricalVisualization {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub;
    ros::Publisher pointCloud_Pub;
    sensor_msgs::ImagePtr msg;
public:
    CylindricalVisualization();
    void cameraCb(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::CameraInfoConstPtr& cam_info);
};


#endif //EGOCYLINDRICAL_CYLINDRICALVISUALIZATION_H
