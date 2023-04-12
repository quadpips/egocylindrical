#ifndef EGOCYLINDRICAL_SENSOR_H
#define EGOCYLINDRICAL_SENSOR_H

namespace egocylindrical
{
  namespace utils
  {
    struct SensorCharacteristics
    {
      SensorCharacteristics(const SensorCharacteristics& c):
        publish_update(c.publish_update),
        raytrace(c.raytrace)
        {}
      bool publish_update=false;
      bool raytrace=false;      
    };
    
    class SensorMeasurement : public SensorCharacteristics
    {
    public:
      SensorMeasurement(const SensorCharacteristics sc, const std_msgs::Header header): 
        SensorCharacteristics(sc),
        header(header) 
        {}
      //virtual std_msgs::Header getHeader() const = 0;
      virtual void insert(ECWrapper& cylindrical_points) = 0;
      std_msgs::Header getHeader() const {return header;}
      
    public:
      const std_msgs::Header header;
      const SensorCharacteristics sc_;
    };
    
    
    class SensorInterface
    {
    public:
      using Callback = boost::function<void(SensorMeasurement&) > ;
      void setCallback(Callback cb) {cb_ = cb;}
      
    protected:
      Callback cb_=0;
      
      SensorCharacteristics sc_;
    };
    
    template<typename T>
    class TypedSensorInterface: public SensorInterface
    {
    public:
    };
    
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_SENSOR_H
