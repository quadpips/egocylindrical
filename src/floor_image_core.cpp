#include <egocylindrical/ecwrapper.h>

#include <egocylindrical/floor_image_core.h>
#include <egocylindrical/floor_image_core_inl.h>


namespace egocylindrical
{
    namespace utils
    {
        sensor_msgs::ImagePtr getFloorNormalColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorNormalColoredImageMsg(cylindrical_history, num_threads, preallocated_msg);
        }

        sensor_msgs::ImagePtr getFloorNormalImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorNormalImageMsg<float>(cylindrical_history, num_threads, preallocated_msg);
        }

        sensor_msgs::ImagePtr getFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, 
                                                        sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorLabelColoredImageMsg(cylindrical_history, num_threads, preallocated_msg);
        }

        sensor_msgs::ImagePtr getFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorLabelImageMsg<uint8_t>(cylindrical_history, num_threads, preallocated_msg);
        }

        sensor_msgs::ImagePtr getRawFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorImageMsg<uint16_t>(cylindrical_history, num_threads, preallocated_msg);
        }
        
        sensor_msgs::ImagePtr getFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr preallocated_msg)
        {
            return generateFloorImageMsg<float>(cylindrical_history, num_threads, preallocated_msg);
        }
        
    }
}
