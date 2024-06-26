#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H

#include <egocylindrical/sensor.h>
#include <egocylindrical/depth_image_inserter.h>

#include <egocylindrical/time_filter.h>
#include <image_transport/subscriber_filter.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


namespace egocylindrical
{
  namespace utils
  {
    class DepthImageMeasurement: public SensorMeasurement
    {
    public:
      DepthImageMeasurement(SensorCharacteristics sc, 
                            const sensor_msgs::Image::ConstPtr& image, 
                            const sensor_msgs::CameraInfo::ConstPtr& info, 
                            const sensor_msgs::Image::ConstPtr& labels, 
                            DepthImageInserter* dii):
        SensorMeasurement(sc, info->header),
        image_(image),
        info_(info),
        labels_(labels),
        dii_(dii)
        {}
      
      //virtual std_msgs::Header getHeader() const {return info_->header;}
      
      virtual void insert(ECWrapper& cylindrical_points)
      {
        ros::WallTime temp = ros::WallTime::now();
        dii_->insert(cylindrical_points, image_, info_);
        ROS_INFO_STREAM_NAMED("timing","Adding depth image took " <<  (ros::WallTime::now() - temp).toSec() * 1e3 << "ms");
      }

      
    protected:
      const sensor_msgs::Image::ConstPtr image_;
      const sensor_msgs::Image::ConstPtr labels_;
      const sensor_msgs::CameraInfo::ConstPtr info_;
      utils::DepthImageInserter* dii_;
    };
    
    class DepthImageSensor: public SensorInterface
    {
      ros::NodeHandle pnh_;
      tf2_ros::Buffer& buffer_;
      
      utils::DepthImageInserter dii_;
      
      image_transport::ImageTransport it_;
      image_transport::SubscriberFilter depth_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
      image_transport::SubscriberFilter labels_sub_;

      using TimeFilter_t = TimeFilter<sensor_msgs::CameraInfo>;
      boost::shared_ptr<TimeFilter_t> time_filter_;
      
      using TfFilter = tf2_ros::MessageFilter<sensor_msgs::CameraInfo>;
      boost::shared_ptr<TfFilter> info_tf_filter;
      
      using MsgSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image>;
      boost::shared_ptr<MsgSynchronizer> msg_sync_;
      
      
    public:
      DepthImageSensor(ros::NodeHandle pnh, tf2_ros::Buffer& buffer):
        pnh_(pnh),
        buffer_(buffer),
        dii_(buffer, pnh),
        it_(pnh)
        {}

      
      void init(std::string fixed_frame_id) override
      {
        //Load general parameters
        sc_.init(pnh_);
        
        //sc_.name = depth_topic;
        //sc_.publish_update = true;
        //sc_.raytrace = true;
        
        //Load implementation parameters
        std::string depth_topic="/camera/depth/image_raw", 
                    info_topic= "/camera/depth/camera_info",
                    labels_topic="/camera/steppability/labels";
        pnh_.getParam("image_in", depth_topic );
        pnh_.getParam("info_in", info_topic );
        pnh_.getParam("labels_in", labels_topic );
        
        //Initialize helper classes
        dii_.init(fixed_frame_id);
        
        //Set up publishers/subscribers and any necessary filters
        depth_sub_.subscribe(it_, depth_topic, 3);
        depth_info_sub_.subscribe(pnh_, info_topic, 3);
        labels_sub_.subscribe(it_, labels_topic, 3);

        //Filter out images with duplicate time stamps
        time_filter_ = boost::make_shared<TimeFilter_t>(depth_info_sub_);
        
        // Ensure that the message is transformable
        info_tf_filter = boost::make_shared<TfFilter>(*time_filter_, buffer_, fixed_frame_id, 2, pnh_);

        // Synchronize Image and CameraInfo callbacks
        msg_sync_ = boost::make_shared<MsgSynchronizer>(depth_sub_, *info_tf_filter, labels_sub_, 2);
        msg_sync_->registerCallback(boost::bind(&DepthImageSensor::update, this, _1, _2, _3));
      }
      
    protected:
      void update(const sensor_msgs::Image::ConstPtr& image, 
                  const sensor_msgs::CameraInfo::ConstPtr& info,
                  const sensor_msgs::Image::ConstPtr& labels)
      {
        if(cb_)
        {
          auto m = boost::make_shared<DepthImageMeasurement>(sc_, image, info, labels, &dii_);
          cb_(m);
        }
        else
        {
          ROS_ERROR("No callback defined for DepthImageSensor!");
        }
      }
      
    public:
      using Ptr = std::shared_ptr<DepthImageSensor>;

    };
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H
