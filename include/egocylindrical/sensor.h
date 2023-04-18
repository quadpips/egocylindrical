#ifndef EGOCYLINDRICAL_SENSOR_H
#define EGOCYLINDRICAL_SENSOR_H

namespace egocylindrical
{
  namespace utils
  {
    struct SensorCharacteristics
    {
//       SensorCharacteristics() = default;
// //       SensorCharacteristics(SensorCharacteristics c) = default;
//       SensorCharacteristics(const SensorCharacteristics& c) = default;
      std::string name;
      bool publish_update=false;
      bool raytrace=false;      
    };
    
    class SensorMeasurement : public SensorCharacteristics
    {
    public:
      SensorMeasurement(SensorCharacteristics sc, std_msgs::Header header): 
        SensorCharacteristics(sc),
        header(header) 
        {}
      //virtual std_msgs::Header getHeader() const = 0;
      virtual void insert(ECWrapper& cylindrical_points) = 0;
      std_msgs::Header getHeader() const {return header;}
      
    public:
       std_msgs::Header header;
//       const SensorCharacteristics sc_;
       
       using Ptr = boost::shared_ptr<SensorMeasurement>;
       using ConstPtr = boost::shared_ptr<const SensorMeasurement>;
    };
    
    
    class SensorInterface
    {
    public:
      using Callback = boost::function<void(SensorMeasurement::Ptr) > ;
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
