
#include <egocylindrical/sensor.h>
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
