//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
#include <ros/ros.h>


namespace egocylindrical
{

    void EgoCylindricalPropagator::update(utils::SensorMeasurement& measurement)
    {
        //TODO: Make ecwrapper pointers into local variables
        old_pts_ = wrapper_buffer_.getOld();
        
        // //TODO: Reset between runs so that the reset function returns once reset is complete
        // {
        //     WriteLock lock(reset_mutex_);
        //     if(should_reset_)
        //     {
        //         old_pts_ = nullptr;
        //         should_reset_ = false;
        //     }
        // }
        
        auto measurement_header = measurement.header;
        auto new_stamp = measurement_header.stamp;
        if(old_pts_)
        {
            if(old_pts_->getHeader().stamp > new_stamp)
            {
                //old_pts_ = nullptr;
            }
            else if(old_pts_->getHeader().stamp == new_stamp)
            {
              ROS_WARN_STREAM_NAMED("msg_timestamps","Repeat stamps received! " << new_stamp);
              //return;
            }
        }
        
                
        if(old_pts_)
        {
            ROS_DEBUG_STREAM_NAMED("msg_timestamps","Previous stamp " << old_pts_->getHeader().stamp);
        }

        //TODO: Warn of out-of-order images
        //TODO: If clock jumps back in time, reset egocylinder
        ROS_INFO_STREAM_NAMED("update", "Measurement source: " << measurement.name);
        ROS_DEBUG_STREAM_NAMED("msg_timestamps","Current stamp: " << new_stamp);
        ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Received [" << new_stamp << "] at [" << ros::WallTime::now() << "]");

        ros::WallTime start = ros::WallTime::now();
        
        if(!cfh_.updateTransforms(measurement_header))
        {
            ROS_WARN_STREAM("Failed to update transforms!");
            return;
        }

        std_msgs::Header target_header = cfh_.getTargetHeader();
        
        {
            if(old_pts_)
            {
                if(measurement.raytrace)
                {
                    new_pts_ = wrapper_buffer_.getNew(); // adds nullptr
                    try
                    {
                        pp_.transform(*old_pts_, *new_pts_, target_header, config_.num_threads);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                        return; //TODO: After a configurable timeout, reset
                    }
                }
                else
                {
                    new_pts_ = wrapper_buffer_.reuseOld();
                }
            }
            else
            {
                new_pts_ = wrapper_buffer_.getNew();
                new_pts_->setHeader(target_header);
            }
            wrapper_buffer_.releaseOld();
        }

        if(propagated_ec_pub_.getNumSubscribers())
        {
            ros::WallTime t1 = ros::WallTime::now();
            utils::ECWrapper ec_copy = *new_pts_;
            ros::WallTime t2 = ros::WallTime::now();
            utils::ECMsgConstPtr msg = ec_copy.getEgoCylinderPointsMsg();
            ros::WallTime t3 = ros::WallTime::now();
            propagated_ec_pub_.publish(msg);
            ros::WallTime t4 = ros::WallTime::now();
            ROS_DEBUG_STREAM_NAMED("timing", "Time to copy propagated points: " <<  (t2 - t1).toSec() * 1e3 << "ms");
            ROS_DEBUG_STREAM_NAMED("timing", "Time to get message: " <<  (t3 - t2).toSec() * 1e3 << "ms");
            ROS_DEBUG_STREAM_NAMED("timing", "Time to publish: " <<  (t4 - t3).toSec() * 1e3 << "ms");
            ROS_DEBUG_STREAM_NAMED("timing", "Total time to copy & publish propagated points: " <<  (t4 - t1).toSec() * 1e3 << "ms");
        }

        {
            {
                measurement.insert(*new_pts_);
            }

            if(measurement.publish_update)
            {
                if(ec_pub_.getNumSubscribers() > 0 && shouldPublish(new_pts_))
                {
                // TODO: if no one is subscribing, we can propagate the points in place next time (if that turns out to be faster)
                utils::ECMsgConstPtr msg = new_pts_->getEgoCylinderPointsMsg();

                ec_pub_.publish(msg);
                ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[egocylinder] Sent [" << msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
                published(new_pts_);
                }

                if(info_pub_.getNumSubscribers() > 0)
                {
                    info_pub_.publish(new_pts_->getEgoCylinderInfoMsg());
                }
            }
        }


        wrapper_buffer_.update();
        
        ROS_DEBUG_STREAM_NAMED("timing", "Total time: " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
    }
    
    void EgoCylindricalPropagator::connectCB()
    {
        // If no one is listening, we can propagate points in place
        if (ec_pub_.getNumSubscribers() == 0)
        {
            
        }
    }
    
    void EgoCylindricalPropagator::reset()
    {
        // WriteLock lock(reset_mutex_);
        // should_reset_ = true;
        wrapper_buffer_.reset();
    }
    
    void EgoCylindricalPropagator::configCB(const egocylindrical::PropagatorConfig &config, uint32_t level)
    {
        WriteLock lock(config_mutex_);
     
        ROS_INFO_STREAM("Updating propagator config: height=" << config.height << ", width=" << config.width << ", vfov=" << config.vfov << ", can_width=" << config.can_width
        << ", v_offset=" << config.v_offset << ", cyl_radius=" << config.cyl_radius);
        config_ = config;
    }
    
    
    bool EgoCylindricalPropagator::init()
    {
        reconfigure_server_->setCallback(boost::bind(&EgoCylindricalPropagator::configCB, this, _1, _2));
        
        // Get topic names
        std::string points_topic="egocylindrical_points", 
                    filtered_pc_topic="filtered_points", 
                    egocylinder_info_topic="egocylinder_info",
                    propagated_points_topic="propagated_egocylindrical_points";
        fixed_frame_id_ = "odom";

        pnh_.getParam("points_out", points_topic );
        pnh_.getParam("filtered_points", filtered_pc_topic);
        
        pnh_.getParam("fixed_frame_id", fixed_frame_id_);

        // ROS_INFO_STREAM_NAMED("update", "points_topic: " << points_topic);
        // ROS_INFO_STREAM_NAMED("update", "filtered_pc_topic: " << filtered_pc_topic);
        // ROS_INFO_STREAM_NAMED("update", "egocylinder_info_topic: " << egocylinder_info_topic);
        // ROS_INFO_STREAM_NAMED("update", "propagated_points_topic: " << propagated_points_topic);
        // ROS_INFO_STREAM_NAMED("update", "fixed_frame_id_: " << fixed_frame_id_);


        cfh_.init();
        wrapper_buffer_.init();

        
        reset_sub_ = nh_.subscribe<std_msgs::Empty>("reset", 1, [this](const std_msgs::Empty::ConstPtr&) { wrapper_buffer_.reset(2); });

        // Setup publishers
        ros::SubscriberStatusCallback image_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);        
        ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(points_topic, 1, image_cb, image_cb);
        
        //ros::SubscriberStatusCallback pc_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_pc_topic, 3);
        info_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(egocylinder_info_topic, 1);

        propagated_ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(propagated_points_topic, 3);

        auto seq_cb = [this](utils::SensorMeasurement::Ptr measurement)
        {
          this->update(*measurement);
        };

        sensors_.init(fixed_frame_id_, seq_cb);
        pp_.init(fixed_frame_id_);
        
        return true;
    }

    EgoCylindricalPropagator::EgoCylindricalPropagator(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh),
        buffer_(),
        tf_listener_(buffer_),
        cfh_(buffer_, pnh),
        sensors_(pnh, buffer_),
        pp_(buffer_),
        wrapper_buffer_(config_),
        should_reset_(false)
    {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        
    }
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
