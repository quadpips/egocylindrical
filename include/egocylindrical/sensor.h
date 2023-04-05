#ifndef EGOCYLINDRICAL_SENSOR_H
#define EGOCYLINDRICAL_SENSOR_H

namespace egocylindrical
{
  namespace utils
  {
    class SensorMeasurement
    {
    public:
      SensorMeasurement(const std_msgs::Header& header): header_(header) {}
      //virtual std_msgs::Header getHeader() const = 0;
      virtual void insert(ECWrapper& cylindrical_points) = 0;
      const std_msgs::Header& getHeader() const {return header_;}
      
    public:
      const std_msgs::Header& header_;
    };
    
    
    class SensorInterface
    {
    public:
      using Callback = boost::function<void(SensorMeasurement&) > ;
      void setCallback(Callback cb) {cb_ = cb;}
      
    protected:
      Callback cb_=0;
    };
    
    template<typename T>
    class TypedSensorInterface: public SensorInterface
    {
    public:
    };
    
    
  } //end namespace utils
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_SENSOR_H
