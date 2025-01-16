//
// Created by root on 2/5/18.
//

#include <egocylindrical/floor_image_generator.h>
// #include <egocylindrical/floor_image_core.h>
#include <egocylindrical/can_image_core.h>


namespace egocylindrical
{



    EgoCylinderFloorImageGenerator::EgoCylinderFloorImageGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh),
        it_(nh_)
    {
        std::cout<<"Egocylindrical Floor Image Node Initialized"<<std::endl;
    }
    
    bool EgoCylinderFloorImageGenerator::init()
    {
        use_raw_ = false;
        std::string image_topic = "image", can_image_topic = "can_image";
        
        pnh_.getParam("use_raw", use_raw_ );
        
        pnh_.getParam("image_topic", image_topic );
        pnh_.getParam("can_image_topic", can_image_topic );
        
        
        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh_);
        reconfigure_server_->setCallback(boost::bind(&EgoCylinderFloorImageGenerator::configCB, this, _1, _2));
        

        image_transport::SubscriberStatusCallback image_cb = boost::bind(&EgoCylinderFloorImageGenerator::ssCB, this);
        {
            Lock lock(connect_mutex_);
            can_im_pub_ = it_.advertise(can_image_topic, 2, image_cb, image_cb);
        }
        
        return true;
    }
    
    void EgoCylinderFloorImageGenerator::configCB(const ConfigType &config, uint32_t level)
    {
      //Num_threads not actually used right now, so not important to lock
      //WriteLock lock(config_mutex_);
      
      ROS_INFO_STREAM("Updating Floor Image Generator config: num_threads=" << config.num_threads);
      num_threads_ = config.num_threads;
    }
    
    void EgoCylinderFloorImageGenerator::ssCB()
    {
        
        //std::cout << (void*)ec_sub_ << ": " << im_pub_.getNumSubscribers() << std::endl;
        Lock lock(connect_mutex_);
        if(can_im_pub_.getNumSubscribers()>0)
        {
            if((void*)ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderFloorImageGenerator::ecPointsCB, this);
                ROS_INFO("FloorImage Generator Subscribing");

            }
      
        }
        else
        {
            ec_sub_.shutdown();
            ROS_INFO("FloorImage Generator Unsubscribing");

        }
    }

    
    void EgoCylinderFloorImageGenerator::ecPointsCB(const egocylindrical::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        ROS_DEBUG("Received EgoCylinderPoints msg");
        
        ROS_DEBUG_STREAM_NAMED("msg_timestamps.detailed","[range_image_generator] Received [" << ec_msg->header.stamp << "] at [" << ros::WallTime::now() << "]");
        
        bool gen_can_image = can_im_pub_.getNumSubscribers() > 0;
        
        utils::ECWrapper ec_pts(ec_msg);
        
        if(gen_can_image)
        {
          ros::WallTime start = ros::WallTime::now();
          
          sensor_msgs::Image::ConstPtr image_ptr = use_raw_ ? utils::getRawCanImageMsg(ec_pts, num_threads_, preallocated_msg_) : utils::getCanImageMsg(ec_pts, num_threads_, preallocated_msg_);
          
          ROS_DEBUG_STREAM_NAMED("timing","Generating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          
          ROS_DEBUG("publish egocylindrical image");
          
          can_im_pub_.publish(image_ptr);
          
          start = ros::WallTime::now();
          preallocated_can_msg_= boost::make_shared<sensor_msgs::Image>();
          preallocated_can_msg_->data.resize(image_ptr->data.size()); //We initialize the image to the same size as the most recently generated image
          ROS_DEBUG_STREAM_NAMED("timing","Preallocating can image took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        }
        
    }




}
