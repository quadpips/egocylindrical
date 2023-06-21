#ifndef EGOCYLINDER_PARAMETER_GETTER_H
#define EGOCYLINDER_PARAMETER_GETTER_H

#include <string>
#include <ros/node_handle.h>
#include <ros/console.h>

namespace egocylindrical
{
  namespace utils
  {
      template <typename T>
      bool getParam(ros::NodeHandle nh, std::string param_name, T& value)
      {
          if(nh.getParam(param_name, value))
          {
              ROS_INFO_STREAM("Successfully loaded parameter [" << param_name << "]=" << value);
              return true;
          }
          else
          {
              ROS_ERROR_STREAM("Unable to find parameter [" << param_name << "]! Full namespace=" << nh.getNamespace() + "/" + param_name);
              return false;
          }
      }

  }
}


#endif //EGOCYLINDER_PARAMETER_GETTER_H
