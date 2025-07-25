#ifndef EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H
#define EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H


//#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/EgoCylinderPoints.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <dynamic_reconfigure/server.h>

#include <rclcpp/rclcpp.hpp>
#include <boost/thread/mutex.hpp>


namespace egocylindrical
{


    class RangeImageDilator
    {
        using Mutex = boost::mutex;
        using Lock = Mutex::scoped_lock;

        ros::NodeHandle nh_, pnh_;
        image_transport::ImageTransport it_;

        image_transport::SubscriberFilter im_sub_, can_im_sub_;
        message_filters::Subscriber<sensor_msgs::msg::Image> ec_sub_;

        bool use_egocan_;

        typedef message_filters::TimeSynchronizer<sensor_msgs::msg::Image, egocylindrical::EgoCylinderPoints> synchronizer;
        boost::shared_ptr<synchronizer> timeSynchronizer;
        typedef message_filters::TimeSynchronizer<sensor_msgs::msg::Image, egocylindrical::EgoCylinderPoints, sensor_msgs::msg::Image> can_synchronizer;
        boost::shared_ptr<can_synchronizer> timeSynchronizerWithCan;

        ros::Publisher im_pub_;

        Mutex connect_mutex_;

    public:

        RangeImageDilator(ros::NodeHandle& nh, ros::NodeHandle& pnh);

        bool init();

        void ssCB();



    private:

        void imageCB(const sensor_msgs::msg::Image::ConstPtr& image, const egocylindrical::EgoCylinderPoints::ConstPtr& info, const sensor_msgs::msg::Image::ConstPtr& can_image);

    };

} // ns egocylindrical



#endif //EGOCYLINDRICAL_RANGE_IMAGE_DILATOR_H

