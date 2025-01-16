#ifndef FLOOR_IMAGE_CORE_H
#define FLOOR_IMAGE_CORE_H

#include <egocylindrical/ecwrapper.h>

#include <sensor_msgs/Image.h>


namespace egocylindrical
{
    
    namespace utils
    {
        // normals
        sensor_msgs::ImagePtr getFloorNormalImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        // labels
        sensor_msgs::ImagePtr getFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        sensor_msgs::ImagePtr getFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        // depth
        sensor_msgs::ImagePtr getRawFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);

        sensor_msgs::ImagePtr getFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg=nullptr);
        
    }
}

#endif //FLOOR_IMAGE_CORE_H
