#include <egocylindrical/range_image_inflator_impl.h>
#include <egocylindrical/range_image_inflator_impl2.h>

namespace egocylindrical
{  
    void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const float unknown_value)
    {
      inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    }

    void inflateRawRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const uint16_t unknown_value)
    {
      inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    }

} //end namespace egocylindrical
