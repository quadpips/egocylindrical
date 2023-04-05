
#include <egocylindrical/sensor.h>
#include <ros/message_traits.h>


namespace ros
{
namespace message_traits
{

  struct IsMessage< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };
    
   
  struct HasHeader< ::egocylindrical::utils::SensorMeasurement >
    : TrueType
    { };

}
}
