//
// Created by root on 2/5/18.
//

#ifndef EGOCYLINDRICAL_EGOCYLINDRICAL_H
#define EGOCYLINDRICAL_EGOCYLINDRICAL_H


#include <egocylindrical/coordinate_frame_helper.h>


#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <tf2_ros/transform_listener.h>


#include <dynamic_reconfigure/server.h>
#include <egocylindrical/PropagatorConfig.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

//#include <egocylindrical/sensor.h>
//#include <egocylindrical/laser_scan_sensor.h>
//#include <egocylindrical/depth_image_sensor.h>
#include <egocylindrical/point_propagator.h>
//#include <egocylindrical/non_message_sequencer_filter.h>
#include <egocylindrical/sensor_collection.h>


namespace egocylindrical
{

class EgoCylindricalPropagator{


  
private:

    typedef boost::shared_mutex Mutex;
    typedef boost::unique_lock< Mutex > WriteLock;
    typedef boost::shared_lock< Mutex > ReadLock;

    Mutex config_mutex_, reset_mutex_;

    utils::ECWrapperPtr new_pts_, old_pts_, transformed_pts_, next_pts_;
    
    ros::NodeHandle nh_, pnh_;
    
protected:
  tf2_ros::Buffer buffer_;
  std::string fixed_frame_id_;
  
private:
    tf2_ros::TransformListener tf_listener_;
    utils::CoordinateFrameHelper cfh_;

    //image_transport::ImageTransport it_;
    utils::SensorCollection sensors_;
    //utils::LaserScanSensor lss_;
    //utils::DepthImageSensor dis_;
    utils::PointPropagator pp_;
    
    ros::Publisher ec_pub_, pc_pub_, info_pub_;
    ros::Subscriber reset_sub_;
    
    //egocylindrical::TimeSequencer<utils::SensorMeasurement> seq_;

    
    egocylindrical::PropagatorConfig config_;
    typedef dynamic_reconfigure::Server<egocylindrical::PropagatorConfig> ReconfigureServer;
    std::shared_ptr<ReconfigureServer> reconfigure_server_;
    
    bool should_reset_;


    void propagateHistory(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, std_msgs::Header new_header);
    
    void connectCB();
    
    void configCB(const egocylindrical::PropagatorConfig &config, uint32_t level);
    
    virtual bool shouldPublish(const utils::ECWrapperPtr& points) { return true;}
    virtual void published(utils::ECWrapperPtr& points) { }


    
public:
    EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~EgoCylindricalPropagator();
    
    void update(utils::SensorMeasurement& measurement);
    sensor_msgs::PointCloud2  getPropagatedPointCloud();
    sensor_msgs::Image::ConstPtr getRawRangeImage();

    virtual bool init();
    void reset();

};


} //end namespace egocylindrical


#endif //EGOCYLINDRICAL_EGOCYLINDRICAL_H
