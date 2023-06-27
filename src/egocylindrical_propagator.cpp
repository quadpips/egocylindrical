//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>
#include <ros/ros.h>


namespace egocylindrical
{
    
    utils::ECParams getParams(const egocylindrical::PropagatorConfig &config)
    {
      utils::ECParams params;
      params.height = config.height;
      params.width = config.width;
      params.vfov = config.vfov;
      params.can_width = config.can_width;
      params.v_offset = config.v_offset;
      params.cyl_radius = config.cyl_radius;
      return params;
    }
    
    namespace utils
    {
      utils::ECWrapperPtr getECWrapper(const egocylindrical::PropagatorConfig &config, bool allocate_arrays=false)
      {
        return utils::getECWrapper(getParams(config), allocate_arrays);
      }
    }

    void EgoCylindricalPropagator::update(utils::SensorMeasurement& measurement)
    {
        //TODO: Reset between runs so that the reset function returns once reset is complete
        {
            WriteLock lock(reset_mutex_);
            if(should_reset_)
            {
                old_pts_ = nullptr;
                should_reset_ = false;
            }
        }
        
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
        
        // NOTE: It may be better to only create the necessary wrappers once and just 'swap' the msg_ pointers
        new_pts_ = next_pts_;
        bool allocate_next = !old_pts_ || old_pts_->isLocked();
        
        if(!cfh_.updateTransforms(measurement_header))
        {
            ROS_WARN_STREAM("Failed to update transforms!");
            return;
        }
//         if(old_pts_) // || measurement.raytrace)
//         {
//           if(!cfh_.updateTransforms(measurement_header))
//           {
//               ROS_WARN_STREAM("Failed to update transforms!");
//               return;
//           }
//         }
//         else
//         {
//           ROS_WARN_STREAM("Ignoring measurement");
//           return;
//         }
        
        std_msgs::Header target_header = cfh_.getTargetHeader();
        
        #pragma omp parallel sections num_threads(2) if(allocate_next)
        {
          #pragma omp section
          {
            try
            {
                if(old_pts_ && measurement.raytrace)
                {
                    pp_.transform(*old_pts_, *new_pts_, target_header, config_.num_threads);
                }
                else
                {
                    new_pts_->setHeader(target_header);
                }
                
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
            }
            //TODO: Decide how to handle transform failure
            
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
          
          #pragma omp section
          {
            ros::WallTime start = ros::WallTime::now();
            
            //Lock mutex
            ReadLock lock(config_mutex_);
            
            if(allocate_next)
            {
                ROS_DEBUG_STREAM("Create new ECWrapper for next time");
                next_pts_ = utils::getECWrapper(config_);
            }
            else
            {
                ROS_DEBUG_STREAM("Reuse old ECWrapper for next time");
                std::swap(next_pts_,old_pts_);

                next_pts_->init(getParams(config_), true);
            }
            
            if((!next_pts_) || next_pts_->getNumPts()==0 || (!next_pts_->msg_) || next_pts_->msg_->points.data.size()==0)
            {
              int tempa = 0;  //Just a place to put a breakpoint
            }
            
            ROS_DEBUG_STREAM_NAMED("timing", "Creating new datastructure took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          }
          
        
        }
        
        std::swap(new_pts_, old_pts_);  
        
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
        WriteLock lock(reset_mutex_);
        should_reset_ = true;
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
        
//         transformed_pts_ = utils::getECWrapper(config_, true);
        
        next_pts_ = utils::getECWrapper(config_);
                
        
        // Get topic names
        std::string points_topic="egocylindrical_points", filtered_pc_topic="filtered_points", egocylinder_info_topic="egocylinder_info";
        fixed_frame_id_ = "odom";

        pnh_.getParam("points_out", points_topic );
        pnh_.getParam("filtered_points", filtered_pc_topic);
        
        pnh_.getParam("fixed_frame_id", fixed_frame_id_);

        cfh_.init();
        
        reset_sub_ = nh_.subscribe<std_msgs::Empty>("reset", 1, [this](const std_msgs::Empty::ConstPtr&) { reset(); });

        // Setup publishers
        ros::SubscriberStatusCallback image_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);        
        ec_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(points_topic, 1, image_cb, image_cb);
        
        //ros::SubscriberStatusCallback pc_cb = boost::bind(&EgoCylindricalPropagator::connectCB, this);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(filtered_pc_topic, 3);
        info_pub_ = nh_.advertise<egocylindrical::EgoCylinderPoints>(egocylinder_info_topic, 1);

        auto seq_cb = [this](utils::SensorMeasurement::Ptr measurement)
        {
          this->update(*measurement);
        };

//         seq_.setCallback(boost::bind(&EgoCylindricalPropagator::update, this, _1));
        //seq_.setCallback(seq_cb);
        
        //auto seq_input = [this](utils::SensorMeasurement::Ptr measurement)
        //{
        //  seq_.add(measurement);
        //};
        
        sensors_.init(fixed_frame_id_, seq_cb);
        
//         lss_.setCallback(boost::bind(&EgoCylindricalPropagator::update, seq_, _1));
        //lss_.setCallback(seq_input);
//         dis_.setCallback(boost::bind(&EgoCylindricalPropagator::update, seq_, _1));
        //dis_.setCallback(seq_input);
        //lss_.init(fixed_frame_id_);
        //dis_.init(fixed_frame_id_);
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
//         lsi_(buffer_, pnh),
//        it_(nh),
        //lss_(buffer_, pnh),
        //dis_(buffer_, pnh, it_),
        pp_(buffer_),
        //seq_(ros::Duration(0.005), ros::Duration(0.01), 10),
        should_reset_(false)
    {
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        
    }
    
    EgoCylindricalPropagator::~EgoCylindricalPropagator()
    {
      
    }
      


}
