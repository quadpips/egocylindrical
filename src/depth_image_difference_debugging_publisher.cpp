#include <egocylindrical/depth_image_difference_debugging_publisher.h>
#include <ros/ros.h>
#include <vector>

namespace egocylindrical
{
    namespace utils
    {

        void DIDiffDebuggingPublisher::init(ros::NodeHandle pnh)
        {
            ros::NodeHandle ppnh(pnh, "debugging");

            auto add_publisher = [this, &ppnh](std::string name, auto f)
            {
                DIDiffDebugging d;
                using L = decltype(*f(d));

                ros::Publisher pub = ppnh.advertise<L>(name, 2);

                auto pub_func = [f,pub](DIDiffDebugging data)
                {
                    auto msg = f(data);
                    if(msg)
                    {
                        pub.publish(msg);
                    }
                };

                publishers_.emplace(name, pub_func);
            };

            add_publisher("depth_image", [](DIDiffDebugging data){return data.depth_image;});
            add_publisher("depth_diff_image", [](DIDiffDebugging data){return data.depth_diff_image;});
            add_publisher("reprojected_depth_image", [](DIDiffDebugging data){return data.reproj_depth_image;});
            add_publisher("range_image", [](DIDiffDebugging data){return data.range_image;});
            add_publisher("point_cloud", [](DIDiffDebugging data){return data.point_cloud;});
            add_publisher("dilated_range_image", [](DIDiffDebugging data){return data.dilated_range_image;});
            add_publisher("dilated_point_cloud", [](DIDiffDebugging data){return data.dilated_point_cloud;});
            add_publisher("marker_array", [](DIDiffDebugging data){return data.marker_array;});
            
        }

        void DIDiffDebuggingPublisher::publish(const DIDiffDebugging& data)
        {
            for(std::map<std::string, func_type>::iterator it = publishers_.begin(); it != publishers_.end(); ++it)
            {
//                 it->first
                it->second(data);
            }
        }

        void DIDiffDebuggingPublisher::reset()
        {

        }

    }
}
