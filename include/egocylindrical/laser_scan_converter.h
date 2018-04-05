//
// Created by root on 4/3/18.
//

#ifndef EGOCYLINDRICAL_LASER_SCAN_CONVERTER_H
#define EGOCYLINDRICAL_LASER_SCAN_CONVERTER_H

#include "utils.h"
#include <math.h>
namespace egocylindrical
{
    namespace utils
    {
        inline
        void stixel_to_LaserScan(utils::ECStixelPtr stixel, sensor_msgs::LaserScan& scan, int cylindrical_width)
        {
            scan.angle_min = -M_PI;
            scan.angle_max = M_PI;
            scan.angle_increment = M_PI * 2 / cylindrical_width;
            scan.ranges.resize(cylindrical_width, utils::dNaN);
            scan.range_min = 0;
            scan.range_max = 6.5;
            scan.time_increment = 0;
            std::vector<float> x(*stixel->getX());
            std::vector<float> z(*stixel->getZ());
            for(int i = 0; i  < cylindrical_width; ++i)
            {
                int depth;
                float theta;
                int index;
                if(x[i] == x[i] && z[i] == z[i])
                {
                    depth = hypot(x[i], z[i]);
                    theta = atan2(z[i], x[i]);
                    float temp  = (theta - scan.angle_min) / scan.angle_increment + 3 *  cylindrical_width / 4;
                    index = roundf(temp);
                    scan.ranges.at(index % cylindrical_width) = depth;
                }

            }
        }

    }
}


#endif //EGOCYLINDRICAL_LASER_SCAN_CONVERTER_H
