//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
//#include <tf/LinearMath/Matrix3x3.h>
//#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

//#include <valgrind/callgrind.h>

namespace egocylindrical
{



    void EgoCylindricalPropagator::propagateHistory(utils::ECStixel& old_pnts, utils::ECStixel& new_pnts, std_msgs::Header new_header)
    {
        ros::WallTime start = ros::WallTime::now();
        
        std_msgs::Header old_header = old_pnts.getHeader();
        
        new_pnts.setHeader(new_header);
        
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                "odom");
        
        ROS_INFO_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                
        
        start = ros::WallTime::now();        
        utils::transformPoints(old_pnts, trans);
        ROS_INFO_STREAM_NAMED("timing", "Transform points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        utils::addPoints(new_pnts, old_pnts, ccc_, false);
        ROS_INFO_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");

    }


    void EgoCylindricalPropagator::addCurrentStixel(utils::ECStixel& cylindrical_points, const stixel_estimator::stixelListMsg::ConstPtr& stixels, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        model_t.fromCameraInfo(cam_info);
        
        utils::addStixel(cylindrical_points, stixels, ccc_, model_t);
        
    }


    void EgoCylindricalPropagator::update(const stixel_estimator::stixelListMsg::ConstPtr& stixels, const sensor_msgs::CameraInfo::ConstPtr& cam_info)
    {
        ros::WallTime start = ros::WallTime::now();

        new_pts_ = utils::getECStixel(cylinder_height_,cylinder_width_);

        try
        {
            if(old_pts_)
            {
                EgoCylindricalPropagator::propagateHistory(*old_pts_, *new_pts_, stixels->header);
                ROS_INFO_STREAM("Propagation took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
//                 ROS_INFO_STREAM(old_pts_->getX()->at(1));
            }
//             std::vector<float> x = old_pts_->getX();
//             ROS_INFO_STREAM(x[1]);
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
        }
        
        
        
        start = ros::WallTime::now();
        
        EgoCylindricalPropagator::addCurrentStixel(*new_pts_, stixels, cam_info);
        ROS_INFO_STREAM("Adding depth image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        new_pts_->generateStixel(model_t, baseline_);
        
        // Visualization
        if(enableMarkerPublish_)
        {
            std::vector<stixel_estimator::stixelMsg>* pub_stixels = new_pts_->getStixels();
            visualization_msgs::Marker current_msg;
            utils::stixels_to_rviz ( *new_pts_, current_msg );
            marker_pub.publish ( current_msg );
        }
        
        // Pub ego stixel
        if(ec_pub_.getNumSubscribers() > 0)
        {
            start = ros::WallTime::now();
            
            stixel_estimator::stixelListMsg::ConstPtr msg = new_pts_->getECStixelMsg();
            ROS_INFO_STREAM("Copying EgoCylinderPoints took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
            
            ec_pub_.publish(msg);
        }



        start = ros::WallTime::now();
        sensor_msgs::LaserScan scan;

        scan.header = stixels->header;
        scan.header.frame_id = "base_link";
        utils::stixel_to_LaserScan(new_pts_, scan, cylinder_width_);
        ROS_INFO_STREAM("Converting to laserscan took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        laser_pub.publish(scan);


        
        std::swap(new_pts_, old_pts_);  
        
    }
    
    void EgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (ec_pub_.getNumSubscribers() == 0)
        {
            
        }
    }
    
    // TODO: add dynamic reconfigure for cylinder height/width, vfov, etc
    void EgoCylindricalPropagator::init()
    {
        
        double pi = std::acos(-1);
        hfov_ = 2*pi;
        vfov_ = pi/2;        
        
        cylinder_width_ = 2048*4;
        cylinder_height_ = 320;
        
        ccc_ = utils::CylindricalCoordsConverter(cylinder_width_, cylinder_height_, hfov_, vfov_);
        
        
        // Get topic names
        std::string stixel_topic="/stixels", info_topic= "/multisense_sl/camera/left/camera_info", pub_topic="egocylindrical_stixel";
        
        pnh_.getParam("stixel_in", stixel_topic );
        pnh_.getParam("info_in", info_topic );
        pnh_.getParam("stixel_out", pub_topic );
        
        // Setup publishers
        ros::SubscriberStatusCallback stixel_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);        
        ec_pub_ = nh_.advertise<stixel_estimator::stixelListMsg>(pub_topic, 1, stixel_cb, stixel_cb);
        marker_pub = nh_.advertise<visualization_msgs::Marker> ( "/egocylindrical/visualization_marker", 50 );
        laser_pub = nh_.advertise<sensor_msgs::LaserScan>("/egocylindrical/laserScan", 100);
        
        // Setup subscribers
        stixelSub.subscribe(nh_, stixel_topic, 30);
        stixelInfoSub.subscribe(nh_, info_topic, 30);
        
        // Ensure that CameraInfo is transformable
        info_tf_filter = boost::make_shared<tf_filter>(stixelInfoSub, buffer_, "odom", 30,nh_);
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer = boost::make_shared<synchronizer>(stixelSub, *info_tf_filter, 30);
        timeSynchronizer->registerCallback(boost::bind(&EgoCylindricalPropagator::update, this, _1, _2));
 
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh),
        tf_listener_(buffer_),
        it_(nh)
    {
        
        
    }
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
