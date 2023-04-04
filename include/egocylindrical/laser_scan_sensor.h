#ifndef EGOCYLINDRICAL_LASERSCAN_SENSOR_H
#define EGOCYLINDRICAL_LASERSCAN_SENSOR_H

#include <egocylindrical/sensor.h>
#include <egocylindrical/laser_scan_inserter.h>

namespace egocylindrical
{
  namespace utils
  {
    class LaserScanMeasurement: public SensorMeasurement
    {
    public:
      LaserScanMeasurement(const sensor_msgs::LaserScan::ConstPtr& scan, LaserScanInserter& lsi):
        scan_(scan),
        lsi_(lsi)
        {}
      
      virtual std_msgs::Header getHeader() const {return scan_->header;}
      
      virtual void insert(ECWrapper& cylindrical_points)
      {
        lsi_.insert(cylindrical_points, scan_);
      }

      
    protected:
      const sensor_msgs::LaserScan::ConstPtr scan_;
      utils::LaserScanInserter& lsi_;
    };
    
    class LaserScanSensor: public SensorInterface
    {
      tf2_ros::Buffer& buffer_;
      ros::NodeHandle pnh_;
//       std::string fixed_frame_id_;
      
      LaserScanInserter lsi_;
      
      message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
      
      using TfFilter = tf2_ros::MessageFilter<sensor_msgs::LaserScan>;
      boost::shared_ptr<TfFilter> scan_tf_filter;
      
    public:
      LaserScanSensor(tf2_ros::Buffer& buffer, ros::NodeHandle pnh):
        buffer_(buffer),
        pnh_(pnh),
        lsi_(buffer, pnh)
        {}

      
      void init(std::string fixed_frame_id)
      {
        std::string scan_topic = "scan";
        pnh_.getParam("scan_in", scan_topic);
        scan_sub_.subscribe(pnh_, scan_topic, 3);
        lsi_.init(fixed_frame_id);
//         std::string fixed_frame_id = "odom";
//         pnh_.getParam("fixed_frame_id", fixed_frame_id);

        // Ensure that the scan is transformable
        scan_tf_filter = boost::make_shared<TfFilter>(scan_sub_, buffer_, fixed_frame_id, 2, pnh_);

        scan_tf_filter->registerCallback(boost::bind(&LaserScanSensor::update, this, _1));
      }
      
    protected:
      void update(const sensor_msgs::LaserScan::ConstPtr& scan)
      {
        if(cb_)
        {
          LaserScanMeasurement m(scan, lsi_);
          cb_(m);
        }
        else
        {
          ROS_ERROR("No callback defined for LaserScanSensor!");
        }
      }

    };
    
    
  }
}


#endif //EGOCYLINDRICAL_LASERSCAN_SENSOR_H
