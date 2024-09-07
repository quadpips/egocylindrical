#ifndef RANGE_IMAGE_INFLATOR_IMPL_H
#define RANGE_IMAGE_INFLATOR_IMPL_H

#include <cstdint>
#include <egocylindrical/ecwrapper.h>
#include <sensor_msgs/Image.h>

namespace egocylindrical
{

    void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const float unknown_value);
    void inflateRawRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const uint16_t unknown_value);

}

#endif //RANGE_IMAGE_INFLATOR_IMPL_H