#include <egocylindrical/sensor_collection.h>

#include <egocylindrical/parameter_getter.hpp>

#include <egocylindrical/depth_image_sensor.h>
//#include <egocylindrical/laser_scan_sensor.h>

#include <stdexcept>
#include <string>

//These 2 only needed for printing sensor source names
#include <sstream>
#include <iterator>

#include <ros/node_handle.h>
#include <ros/console.h>
#include <tf2_ros/buffer.h>
//#include <image_transport/image_transport.h>



namespace egocylindrical
{
    namespace utils 
    {
        SensorInterface::Ptr createSensor(ros::NodeHandle nh, std::string name, tf2_ros::Buffer& buffer)
        {
            ROS_INFO_STREAM("Creating sensor [" << name << "]");
            auto sensor_nh = ros::NodeHandle(nh, name);
            
            SensorInterface::Ptr sensor;
            std::string sensor_type;
            if(!sensor_nh.getParam("type", sensor_type))
            {
                throw std::runtime_error("Type [" + sensor_type + "] is not defined for Sensor! [" + name + "]");
            }
            
            //This isn't possible w/ C++11 apparently
            //auto make_sensor = [name, buffer_&, sensor_nh&](){ std::make_shared<T>(name, buffer_, sensor_nh); };
            
            if(sensor_type == "laser")
            {
                //sensor = std::dynamic_pointer_cast<SensorInterface>(std::make_shared<LaserScanSensor>(sensor_nh, buffer));
            }
            else if(sensor_type == "depth")
            {
                sensor = std::make_shared<DepthImageSensor>(sensor_nh, buffer);
            }
            else
            {
                throw std::runtime_error("Sensor type [" + sensor_type + "] not recognized!");
            }
            
            return sensor;
        }


        SensorCollection::SensorCollection(ros::NodeHandle pnh, tf2_ros::Buffer& buffer):
            buffer_(buffer),
            pnh_(pnh)
        {
        
        
        
        }
        
        
        
        
        
        
        bool SensorCollection::init(std::string fixed_frame_id, callback_t& f)
        {
            fixed_frame_id_ = fixed_frame_id;
            
            //read configuration from parameter server
            auto sensor_root_nh = ros::NodeHandle(pnh_, "sensors");
            
            auto get_sensor_names1 = [ ](ros::NodeHandle nh)
            {
                std::vector<std::string> keys;
                nh.getParamNames(keys);
                
                const std::string sensor_ns = nh.getNamespace();
                const size_t ns_len = sensor_ns.length();
                
                std::vector<std::string> names;
                
                for(const auto& key : keys)
                {
                    if(key.rfind(sensor_ns,0) != 0)
                    {
                        continue;
                    }
                    //Extract just the next level of the parameter namespace
                    //Add to a set/list
                }
                
                return names;
            };
            
            auto get_sensor_names2 = [](ros::NodeHandle nh)
            {
                std::vector<std::string> sensor_names;// {"depth", "laser"};
                if(nh.getParam("observation_sources", sensor_names))
                {
                    //Based on https://stackoverflow.com/a/5689061
                    const char* const delim = ", ";
                    std::ostringstream imploded;
                    std::copy(sensor_names.begin(), sensor_names.end(),
                    std::ostream_iterator<std::string>(imploded, delim));
                    ROS_INFO_STREAM("Enabled observation sources: " << delim);
                }

                return sensor_names;
            };
            
            auto get_sensor_names3 = [](ros::NodeHandle nh)
            {
                XmlRpc::XmlRpcValue my_list;
                nh.getParam("observation_sources", my_list);
                ROS_ASSERT(my_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

                std::vector<std::string> sensor_names;
                for (int32_t i = 0; i < my_list.size(); ++i) 
                {
                  ROS_ASSERT(my_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
                  sensor_names.push_back(static_cast<std::string>(my_list[i]));
                }
                return sensor_names;
            };
            
            std::vector<std::string> sensor_names = get_sensor_names3(sensor_root_nh);
            
            for(const auto& name : sensor_names)
            {
                //auto sensor_nh = ros::NodeHandle(sensor_root_nh, name);
                SensorInterface::Ptr sensor = createSensor(sensor_root_nh, name, buffer_);
                if(sensor)
                {
                    sensors_.push_back(sensor);
                }
                else
                {
                    return false;
                }
            }

            if(sensors_.size()>0)
            {
                ROS_INFO_STREAM("Found " << sensors_.size() << " sensors");
            }
            else
            {
                ROS_ERROR_STREAM("Did not find any sensors!");
                return false;
            }
            
            auto seq_input = [this](const utils::SensorMeasurement::ConstPtr& measurement)
            {
              seq_.add(measurement);
            };
            
            //init sensors
            for(auto& sensor : sensors_)
            {
                sensor->setCallback(seq_input);
                sensor->init(fixed_frame_id);
            }
            
            seq_.registerCallback(f);
            
            /*
            SensorInterface::Ptr main_sensor;
            for(auto& sensor : sensors_)
            {
            
            }
            */
            
            return true;
        }
        
        
        bool SensorCollection::setup()
        {
        /*
            auto seq_cb = [this](utils::SensorMeasurement::Ptr measurement)
            {
              this->update(*measurement);
            };

    //         seq_.setCallback(boost::bind(&EgoCylindricalPropagator::update, this, _1));
            seq_.setCallback(seq_cb);
            
            auto seq_input = [this](utils::SensorMeasurement::Ptr measurement)
            {
              seq_.add(measurement);
            };
            

            lss_.setCallback(seq_input);
            dis_.setCallback(seq_input);
            lss_.init(fixed_frame_id_);
            dis_.init(fixed_frame_id_);
            pp_.init(fixed_frame_id_);
        */
            return true;
        }
            

    } //end namespace utils
} //end namespace egocylindrical

