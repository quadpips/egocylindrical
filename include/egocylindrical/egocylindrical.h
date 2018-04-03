//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H

#include <egocylindrical/utils.h>

#include <stixel_estimator/stixelListMsg.h>
#include <stixel_estimator/stixelMsg.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl.h>

#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf2_ros/message_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

namespace egocylindrical
{

class EgoCylindricalPropagator{
private:
    int cylinder_height_;
    int cylinder_width_;
    double hfov_, vfov_;
        
    utils::ECStixelPtr new_pts_, old_pts_;
//     std_msgs::Header old_header_, new_header_;
    image_geometry::PinholeCameraModel model_t;
    

    ros::NodeHandle nh_, pnh_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf_listener_;

    
    image_transport::ImageTransport it_;
    message_filters::Subscriber<stixel_estimator::stixelListMsg> stixelSub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> stixelInfoSub;
    
    typedef tf2_ros::MessageFilter<sensor_msgs::CameraInfo> tf_filter;
    boost::shared_ptr<tf_filter> info_tf_filter;
    
    typedef message_filters::TimeSynchronizer<stixel_estimator::stixelListMsg, sensor_msgs::CameraInfo> synchronizer;
    boost::shared_ptr<synchronizer> timeSynchronizer;
    
    ros::Publisher ec_pub_;
    ros::Publisher marker_pub;
    
    float baseline_ = 0.12;
    bool enableMarkerPublish_ = true;
    
    utils::CylindricalCoordsConverter ccc_;

    void propagateHistory(utils::ECStixel& old_pnts, utils::ECStixel& new_pnts, std_msgs::Header new_header);
    void addCurrentStixel(utils::ECStixel& cylindrical_points, const stixel_estimator::stixelListMsg::ConstPtr& stixels, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    
    void connectCB();
    
    
public:
    EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~EgoCylindricalPropagator();
    
    void update(const stixel_estimator::stixelListMsg::ConstPtr& stixels, const sensor_msgs::CameraInfo::ConstPtr& cam_info);
    sensor_msgs::PointCloud2  getPropagatedPointCloud();
    sensor_msgs::Image::ConstPtr getRawRangeImage();

    void init();

    //pcl::PointCloud<pcl::PointXYZI> getCylindricalPointCloud();
    //pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();


};


}


#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
