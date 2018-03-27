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
#include "egocylindrical.h"
//#include "CylindricalPropagator.h"
//#include <pcl.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <stixel_estimator/stixelMsg.h>
#include <stixel_estimator/stixelListMsg.h>




class CylindricalVisualization {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher pub;
    sensor_msgs::ImagePtr msg;
    ros::Publisher ptPub, ptPub2;
    egocylindrical::EgoCylindricalPropagator propagator_;
public:
    CylindricalVisualization();
    void stixelCb(const stixel_estimator::stixelListMsgConstPtr &stixels, const sensor_msgs::CameraInfoConstPtr& cam_info);
};



#endif //EGOCYLINDRICAL_CYLINDRICALVISUALIZATION_H
