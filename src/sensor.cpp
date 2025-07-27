
#include <egocylindrical/sensor.h>
#include <egocylindrical/parameter_getter.hpp>

//These includes are likely redundant, but included for completeness
#include <string>
// #include <ros/node_handle.h>
// #include <ros/console.h>

namespace egocylindrical
{
  namespace utils
  {
  
      bool SensorCharacteristics::init(rclcpp::Node::SharedPtr node) // ros::NodeHandle sensor_nh
      {
          //Find name of sensor
          auto get_name = [node](std::string& value)
          {
              std::string ns = node->get_namespace();
              auto pos = ns.rfind("/");

              if(pos == std::string::npos)
              {
                  // // ROS_ERROR(_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              
              std::string substr = ns.substr(pos + 1);

              if(substr.empty())
              {
                  // // ROS_ERROR(_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              value = substr;
              return true;
          };

          auto get_param = [node](std::string param_name, auto& value)
          {
              /*
              if(sensor_nh.getParam(param_name, value))
              {
                  // ROS_INFO_STREAM("Successfully loaded parameter [" << param_name << "]=" << value);
                  return true;
              }
              else
              {
                  // // ROS_ERROR(_STREAM("Unable to find parameter [" << param_name << "]! Full namespace=" << sensor_nh.getNamespace() + "/" + param_name);
                  return false;
              }
              */
              return getParam(node, param_name, value);
          };
          
          //sensor_nh.getParam("publish_update", publish_update);
          //sensor_nh.getParam("raytrace", raytrace);
          if(get_name(name) && get_param("publish_update", publish_update) && get_param("raytrace", raytrace))
          {
              return true;
          }
          return false;
      }

  } //end namespace utils
  
} //end namespace egocylindrical
