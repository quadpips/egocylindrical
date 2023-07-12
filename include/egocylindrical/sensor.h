#ifndef EGOCYLINDRICAL_SENSOR_H
#define EGOCYLINDRICAL_SENSOR_H

#include <egocylindrical/ecwrapper.h>
//#include <message_filters/simple_filter.h>
#include <std_msgs/Header.h>
#include <ros/message_event.h>

namespace egocylindrical
{
  namespace utils
  {
    /*
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
            ROS_ERROR_STREAM("Unable to find parameter [" << param_name << "]! Full namespace=" << sensor_nh.getNamespace() + "/" + param_name);
            return false;
        }
    }
    */
    
    struct SensorCharacteristics
    {
      std::string name;
      bool publish_update=false;
      bool raytrace=false;
      bool main=false;
      
      //This is really a factory method, should potentially be moved elsewhere
      virtual bool init(ros::NodeHandle sensor_nh);
      /*
      {
          //Find name of sensor
          auto get_name = [sensor_nh](std::string& value)
          {
              std::string ns = sensor_nh.getNamespace();
              auto pos = x.rfind("/");

              if(pos == std::string::npos)
              {
                  ROS_ERROR_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              
              std::string substr = x.substr(pos + 1);

              if(substr.empty())
              {
                  ROS_ERROR_STREAM("Invalid namespace for sensor! [" << ns << "]");
                  return false;
              }
              value = substr;
              return true;
          };
          
          auto get_param = [sensor_nh](std::string param_name, auto& value)
          {
              return getParam(sensor_nh, param_name, value);
          };
          
          //sensor_nh.getParam("publish_update", publish_update);
          //sensor_nh.getParam("raytrace", raytrace);
          if(get_name(name) && get_param("publish_update", publish_update) && get_param("raytrace", raytrace))
          {
              return true;
          }
          return false;
      }
      */
    };
    
    class SensorMeasurement : public SensorCharacteristics
    {
    public:
      SensorMeasurement(SensorCharacteristics sc, std_msgs::Header header): 
        SensorCharacteristics(sc),
        header(header) 
        {}

      virtual void insert(ECWrapper& cylindrical_points) = 0;
      virtual ros::MessageEvent<SensorMeasurement>::CreateFunction getMessageCreator() = 0;
      std_msgs::Header getHeader() const {return header;}
      
    public:
       std_msgs::Header header;
       
       using Ptr = boost::shared_ptr<SensorMeasurement>;
       using ConstPtr = boost::shared_ptr<const SensorMeasurement>;
    };
    
    template<typename M>
    class TypedSensorMeasurement : public SensorMeasurement
    {
    public:
      TypedSensorMeasurement(SensorCharacteristics sc, std_msgs::Header header):
        SensorMeasurement(sc, header)
        {}
    
      virtual ros::MessageEvent<SensorMeasurement>::CreateFunction getMessageCreator() override
      {
        struct DefaultMessageCreator
        {
          SensorMeasurement::Ptr operator()()
          {
            return boost::make_shared<M>();
          }
        };
        return DefaultMessageCreator();
      }
    };
    
    
    class SensorInterface
    {
    public:
      using Callback = boost::function<void(const SensorMeasurement::ConstPtr&) > ;
      void setCallback(Callback cb) {cb_ = cb;}
      virtual void init(std::string fixed_frame_id)=0;
      
    protected:
      Callback cb_=0;
      
      SensorCharacteristics sc_;
      
    public:
      using Ptr = std::shared_ptr<SensorInterface>;
    };
    
    template<typename M>
    class TypedSensorInterface: public SensorInterface //, public message_filters::SimpleFilter<M>
    {
    public:
      
    };
    
    
  } //end namespace utils
} //end namespace egocylindrical

#include <egocylindrical/sensor_filter.h> //NOTE: This is here to avoid a circular dependency issue

#endif //EGOCYLINDRICAL_SENSOR_H
