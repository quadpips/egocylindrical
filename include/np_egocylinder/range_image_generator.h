#ifndef NP_EGOCYLINDER_RANGE_IMAGE_GENERATOR_H
#define NP_EGOCYLINDER_RANGE_IMAGE_GENERATOR_H


//#include <np_egocylinder/ecwrapper.h>
#include <np_egocylinder/EgoCylinderPoints.h>
#include <np_egocylinder/RangeImageGeneratorConfig.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>

typedef boost::shared_mutex Mutex;
typedef boost::unique_lock< Mutex > WriteLock;
typedef boost::shared_lock< Mutex > ReadLock;



namespace np_egocylinder
{


    class EgoCylinderRangeImageGenerator
    {
        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;
        image_transport::Publisher im_pub_;
        ros::Subscriber ec_sub_;
        bool use_raw_;
        
        //Mutex config_mutex_;
        int num_threads_;
        
        sensor_msgs::Image::Ptr preallocated_msg_;
        
        typedef RangeImageGeneratorConfig ConfigType;
        ConfigType config_;
        typedef dynamic_reconfigure::Server<ConfigType> ReconfigureServer;
        std::shared_ptr<ReconfigureServer> reconfigure_server_;

    public:

        EgoCylinderRangeImageGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        
        bool init();
        
        void configCB(const ConfigType &config, uint32_t level);
        
        void ssCB();



    private:
        
        void ecPointsCB(const EgoCylinderPoints::ConstPtr& ec_msg);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_GENERATOR_H
