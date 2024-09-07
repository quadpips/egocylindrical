//
// Created by root on 2/5/18.
//

#include <egocylindrical/range_image_inflator_generator.h>
#include <egocylindrical/range_image_inflator_impl.h>
//#include <egocylindrical/range_image_inflator_core.h>
#include <algorithm>

// The below are redundant
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <egocylindrical/ecwrapper.h>
#include <sensor_msgs/image_encodings.h>
#include <benchmarking_tools/benchmarking_tools.h>

namespace egocylindrical
{



    RangeImageInflatorGenerator::RangeImageInflatorGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Range Image Node Initialized"<<std::endl;

        
        
    }
    
    bool RangeImageInflatorGenerator::init()
    {
        //use_raw_ = false;
        std::string image_topic = "inflated_image";
                
        pnh_.getParam("image_topic", image_topic );
        
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(boost::bind(&RangeImageInflatorGenerator::configCB, this, _1, _2));
        
        // Synchronize Image and CameraInfo callbacks
        timeSynchronizer_ = boost::make_shared<synchronizer>(im_sub_, ec_sub_, 2);
        timeSynchronizer_->registerCallback(boost::bind(&RangeImageInflatorGenerator::imgCB, this, _1, _2));
        
        image_transport::SubscriberStatusCallback image_cb = boost::bind(&RangeImageInflatorGenerator::ssCB, this);
        {
            Lock lock(connect_mutex_);
            im_pub_ = it_.advertise(image_topic, 2, image_cb, image_cb);
        }
        return true;
    }
    
    void RangeImageInflatorGenerator::configCB(const ConfigType &config, uint32_t level)
    {
      Lock lock(config_mutex_);
      
      ROS_INFO_STREAM("Updating Range Image Inflator config: num_threads=" << config.num_threads << ", inflation_radius=" << config.inflation_radius);
      config_ = config;
    }
    
    void RangeImageInflatorGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_.getNumSubscribers() << std::endl;
        Lock lock(connect_mutex_);
        if(im_pub_.getNumSubscribers()>0)
        {
            if((void*)im_sub_.getSubscriber()) //if currently subscribed... no need to do anything
            {
              
            }
            else
            {
                im_sub_.subscribe(it_, "range_image", 2);
                ec_sub_.subscribe(nh_, "egocylindrical_points", 2);
                
                ROS_INFO("RangeImage Inflator Subscribing");
            }  
        }
        else
        {
            im_sub_.unsubscribe();
            ec_sub_.unsubscribe();
            
            ROS_INFO("RangeImage Inflator Unsubscribing");
        }
    }
    
    
    sensor_msgs::Image::ConstPtr getInflatedRangeImageMsg(const EgoCylinderPoints::ConstPtr& ec_msg, const sensor_msgs::Image::ConstPtr& range_msg, float inflation_radius, float inflation_height, bool conservative, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
    {
      sensor_msgs::ImagePtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : boost::make_shared<sensor_msgs::Image>();
      
      sensor_msgs::Image &new_msg = *new_msg_ptr;
      new_msg.header = range_msg->header;
      new_msg.height = range_msg->height;
      new_msg.width = range_msg->width;
      new_msg.encoding = range_msg->encoding;
      new_msg.is_bigendian = range_msg->is_bigendian;
      new_msg.step = range_msg->step;
      
      size_t size = new_msg.step * new_msg.height;
      
      //ros::WallTime start = ros::WallTime::now();
      
      new_msg.data.resize(size);
      //cv_bridge::CvImage(image->header, sensor_msgs::image_encodings::TYPE_32FC1, new_im_).toImageMsg();
      
      
      // ROS_INFO_STREAM_NAMED("timing","Allocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      
      
      utils::ECConverter converter;
      converter.fromCameraInfo(ec_msg);
      
//       float* inflated_ranges = (float*)new_msg.data.data();
//       std::fill(inflated_ranges, inflated_ranges+converter.getCols(), utils::dNaN);
//       
//       const float* ranges = (float*)range_msg->data.data();
      
      if(range_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      {
        inflateRangeImage(*range_msg, converter, inflation_radius, inflation_height, num_threads, new_msg, utils::dNaN);
      }
      else if(range_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      {
        inflateRawRangeImage(*range_msg, converter, inflation_radius, inflation_height, num_threads, new_msg, 0);
      }
      else
      {
        ROS_ERROR_STREAM("Unsupported image format for inflation: " << range_msg->encoding);
      }
      
      
      
      return new_msg_ptr;
    }

    
    void RangeImageInflatorGenerator::imgCB(const sensor_msgs::Image::ConstPtr& range_msg, const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_DEBUG("Received EgoCylinderPoints msg");
        
        // This may be redundant now
        if(im_pub_.getNumSubscribers() > 0)
        {
          DURATION_DEBUG_STREAM_THROTTLED("inflation", 100, 1);
          ros::WallTime start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          bool conservative = false;
          
          ConfigType config;
          {
            Lock lock(config_mutex_);
            config = config_;
          }
          sensor_msgs::Image::ConstPtr image_ptr = getInflatedRangeImageMsg(ec_msg, range_msg, config.inflation_radius, config.inflation_height/2, conservative, config.num_threads, preallocated_msg_);

          ROS_DEBUG_STREAM_NAMED("timing","Inflating range image by {" << config.inflation_radius << "x" << config.inflation_height/2 << "} took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          ROS_DEBUG("publish egocylindrical image");
          
          im_pub_.publish(image_ptr);
          
          start = ros::WallTime::now();
          preallocated_msg_= boost::make_shared<sensor_msgs::Image>();
          preallocated_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          ROS_DEBUG_STREAM_NAMED("timing","Preallocating image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
        }
        
    }




}
