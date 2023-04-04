#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H

#include <egocylindrical/sensor.h>
#include <egocylindrical/depth_image_inserter.h>

namespace egocylindrical
{
  namespace utils
  {
    class DepthImageMeasurement: public SensorMeasurement
    {
    public:
      DepthImageMeasurement(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr info, DepthImageInserter& dii):
        image_(image),
        info_(info),
        dii_(dii)
        {}
      
      virtual std_msgs::Header getHeader() const {return info_->header;}
      
      virtual void insert(ECWrapper& cylindrical_points)
      {
        dii_.insert(cylindrical_points, image_, info_);
      }

      
    protected:
      const sensor_msgs::Image::ConstPtr image_;
      const sensor_msgs::CameraInfo::ConstPtr info_;
      utils::DepthImageInserter& dii_;
    };
    
    class DepthImageSensor: public SensorInterface
    {
      tf2_ros::Buffer& buffer_;
      ros::NodeHandle pnh_;
//       std::string fixed_frame_id_;
      
      utils::DepthImageInserter dii_;
      
      image_transport::ImageTransport it_;
      image_transport::SubscriberFilter depth_sub_;
      message_filters::Subscriber<sensor_msgs::CameraInfo> depth_info_sub_;
      
      using TfFilter = tf2_ros::MessageFilter<sensor_msgs::CameraInfo>;
      boost::shared_ptr<TfFilter> info_tf_filter;
      
      using MsgSynchronizer = message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>;
      boost::shared_ptr<MsgSynchronizer> msg_sync_;
      
      
    public:
      DepthImageSensor(tf2_ros::Buffer& buffer, ros::NodeHandle pnh, image_transport::ImageTransport it):
        buffer_(buffer),
        pnh_(pnh),
        dii_(buffer, pnh),
        it_(it)
        {}

      
      void init(std::string fixed_frame_id)
      {
        std::string depth_topic="/camera/depth/image_raw", info_topic= "/camera/depth/camera_info";
        dii_.init(fixed_frame_id);
        
        depth_sub_.subscribe(it_, depth_topic, 3);
        depth_info_sub_.subscribe(pnh_, info_topic, 3);

        // Ensure that the scan is transformable
        info_tf_filter = boost::make_shared<TfFilter>(depth_info_sub_, buffer_, fixed_frame_id, 2, pnh_);

        // Synchronize Image and CameraInfo callbacks
        msg_sync_ = boost::make_shared<MsgSynchronizer>(depth_sub_, *info_tf_filter, 2);
        msg_sync_->registerCallback(boost::bind(&DepthImageSensor::update, this, _1, _2));
      }
      
    protected:
      void update(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& info)
      {
        if(cb_)
        {
          DepthImageMeasurement m(image, info, dii_);
          cb_(m);
        }
        else
        {
          ROS_ERROR("No callback defined for DepthImageSensor!");
        }
      }

    };
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_SENSOR_H
