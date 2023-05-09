#ifndef EGOCYLINDRICAL_SENSOR_FILTER_H
#define EGOCYLINDRICAL_SENSOR_FILTER_H


//#include <egocylindrical/sensor.h>
#include <ros/message_traits.h>


namespace ros
{
namespace message_traits
{
  template <>
  struct IsMessage< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };
    
  template <>
  struct HasHeader< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };

}
}

#endif  //EGOCYLINDRICAL_SENSOR_FILTER_H
