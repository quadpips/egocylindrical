#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_DEBUGGING_PUBLISHER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_DEBUGGING_PUBLISHER_H

#include <egocylindrical/depth_image_difference.h>  //Strictly speaking, a forward declaration would suffice, but not worth the potential hassle
#include <ros/ros.h>
#include <map>

namespace egocylindrical
{
    namespace utils
    {
        struct DIDiffDebuggingPublisher
        {
            using func_type = std::function<void(DIDiffDebugging)>;
//             struct Impl
//             {
//                 ros::Publisher pub;
//                 func_type op_f;
//             };

            std::map<std::string, func_type> publishers_;

            void init(ros::NodeHandle pnh);
            void publish(const DIDiffDebugging& info);
            void reset();
        };
    }
}

#endif //egocylindrical_depth_image_difference_debugging_publisher_h
