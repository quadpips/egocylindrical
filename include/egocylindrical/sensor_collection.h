#ifndef EGOCYLINDRICAL_SENSOR_LOADER_H
#define EGOCYLINDRICAL_SENSOR_LOADER_H

//#include <functional> 
//#include <mutex>  //?

#include <egocylindrical/sensor.h>


#include <ros/node_handle.h>
//#include <ros/console.h>

#include <tf2_ros/buffer.h>
//#include <image_transport/image_transport.h>
#include <egocylindrical/time_filter.h>


namespace egocylindrical
{
    namespace utils 
    {
        
        SensorInterface::Ptr createSensor(ros::NodeHandle sensor_nh, std::string name);
        
        class SensorCollection
        {
            std::string fixed_frame_id_;
            
            tf2_ros::Buffer& buffer_;
            ros::NodeHandle pnh_;
            //image_transport::ImageTransport it_;
            TimeFilter<utils::SensorMeasurement> seq_;

            std::vector<SensorInterface::Ptr> sensors_;
            
            using callback_t = const std::function <void (utils::SensorMeasurement::Ptr measurement)>;
            
        public:
        
            SensorCollection(ros::NodeHandle pnh, tf2_ros::Buffer& buffer);
            
            bool init(std::string fixed_frame_id, callback_t& f);
            
            bool setup();
            
        };
    } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_SENSOR_LOADER_H
