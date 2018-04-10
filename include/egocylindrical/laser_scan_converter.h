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
            int resolution_factor = 1;
            scan.angle_min = -M_PI;
            scan.angle_max = M_PI;
            scan.angle_increment = M_PI * 2 / (cylindrical_width * resolution_factor);
            scan.ranges.resize(cylindrical_width * resolution_factor, utils::dNaN);
            scan.range_min = 0.0;
            scan.range_max = 7.0;
            scan.time_increment = 0.0;
            std::vector<float> x(*stixel->getX());
            std::vector<float> z(*stixel->getZ());

            for(int i = 0; i  < cylindrical_width; ++i)
            {
                float depth;
                float theta;
                int index;
                if(x[i] == x[i] && z[i] == z[i])
                {
                    depth = hypot(x[i], z[i]);
                    theta = atan2(z[i], x[i]);
                    index = roundf((theta - scan.angle_min) / (scan.angle_increment * resolution_factor)+ 3 *  cylindrical_width / 4);
                    for(int j = 0; j < resolution_factor; j++)
                    {
                        scan.ranges.at((index % cylindrical_width) *resolution_factor + j) = depth;
                    }
                }

            }
        }

    }
}


#endif //EGOCYLINDRICAL_LASER_SCAN_CONVERTER_H
