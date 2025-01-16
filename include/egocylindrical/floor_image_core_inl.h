#ifndef EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H
#define EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H

#include <egocylindrical/ecwrapper.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <stdint.h>
#include <cmath>

namespace egocylindrical
{
    
    namespace utils
    {
        template <typename T>
        bool isNan(T val)
        {
          return toIEEE754(val).isNan();
        }
        /////////////////////////
        // COLORED LABEL IMAGE //
        /////////////////////////
        void generateFloorLabelColoredImage(const utils::ECWrapper& cylindrical_history, uint32_t* data, const uint32_t unknown_val, int num_threads)
        {            
            ROS_DEBUG("Generating image of cylindrical memory");
            
            const uint8_t* const labels = cylindrical_history.getLabels();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                std::uint8_t r = 0, g = 0, b = 0, a = 0;    // Example: Red color

                if (labels[j] == 0) // VOID
                {
                    // ROS_DEBUG_STREAM_NAMED("labels", " label is void");
                    r = 0; g = 0; b = 0; a = 0;
                } else if (labels[j] == 1) // NONPASS
                {
                    // ROS_DEBUG_STREAM_NAMED("labels", " label is non-passable");
                    r = 255; g = 0; b = 0; a = 255;
                }  else if (labels[j] == 2) // PASS
                {
                    // ROS_DEBUG_STREAM_NAMED("labels", " label is passable");
                    r = 255; g = 255; b = 0; a = 255;
                }  else if (labels[j] == 3) // STEP
                {
                    // ROS_DEBUG_STREAM_NAMED("labels", " label is steppable");
                    r = 0; g = 255; b = 0; a = 255;
                } else
                {
                    ROS_DEBUG_STREAM_NAMED("labels", " label not recognized!");
                }

                std::uint32_t rgba = ((std::uint32_t)a << 24 | (std::uint32_t)b << 16 | (std::uint32_t)g << 8 | (std::uint32_t)r);

                data[j_] = rgba;
            }
        }
        
        sensor_msgs::ImagePtr generateFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                                const uint32_t unknown_val, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::ImagePtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : boost::make_shared<sensor_msgs::Image>();
            
            sensor_msgs::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * 4 * sizeof(uint8_t);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            uint32_t * data = (uint32_t*)new_msg.data.data();
            
            generateFloorLabelColoredImage(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        sensor_msgs::ImagePtr generateFloorLabelColoredImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, 
                                                                sensor_msgs::ImagePtr& preallocated_msg)
        {
          return generateFloorLabelColoredImageMsg(cylindrical_history, sensor_msgs::image_encodings::RGBA8, 
                                                    0, num_threads, preallocated_msg);
        }

        /////////////////
        // LABEL IMAGE //
        /////////////////
        template <typename T>
        void generateFloorLabelImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            ROS_DEBUG("Generating image of cylindrical memory");
            
            const uint8_t* const labels = cylindrical_history.getLabels();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                int j_ = j - start_idx;
                r[j_] = labels[j];
            }
        }
        
        
        template <typename T>
        sensor_msgs::ImagePtr generateFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                            const T unknown_val, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::ImagePtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : boost::make_shared<sensor_msgs::Image>();
            
            sensor_msgs::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateFloorLabelImage<T>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::ImagePtr generateFloorLabelImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr& preallocated_msg);

        template <>
        sensor_msgs::ImagePtr generateFloorLabelImageMsg<uint8_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
          return generateFloorLabelImageMsg<uint8_t>(cylindrical_history, sensor_msgs::image_encodings::TYPE_8UC1, 
                                                      0, num_threads, preallocated_msg);
        }

        /////////////////
        // RANGE IMAGE //
        /////////////////

        template <typename T,uint scale>
        void generateFloorImage(const utils::ECWrapper& cylindrical_history, T* r, const T unknown_val, int num_threads)
        {            
            ROS_DEBUG("Generating image of cylindrical memory");
            
            const float* const y = cylindrical_history.getY();

            const int num_cap_pts = cylindrical_history.getNumCapPts() / 2;
            const int num_cols = cylindrical_history.getCols();
            const int num_pnts = cylindrical_history.getNumPts();

            int start_idx = (num_cols + num_cap_pts);
            #pragma GCC ivdep
            for(int j = start_idx; j < num_pnts; ++j)
            {
                T temp = unknown_val;
                
                float depth = std::abs(y[j]);
                
                if(depth==depth)
                {
                    /* NOTE: It appears that the conversion from float to uint16 is what is preventing this from vectorizing for the uint16 case,
                     * which actually makes sense: the datatypes have different widths, so how could the conversions be vectorized?
                     * It should be possible to vectorize the calculation and have the conversion separate, though unclear whether that would be faster.
                     */
                    temp = depth * scale;                    
                }

                int j_ = j - start_idx;
                r[j_] = temp;
            }
        }
        
        
        template <typename T, uint scale>
        sensor_msgs::ImagePtr generateFloorImageMsg(const utils::ECWrapper& cylindrical_history, const std::string& encoding, 
                                                  const T unknown_val, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
            int width = cylindrical_history.getCanWidth();  
            
            sensor_msgs::ImagePtr new_msg_ptr = (preallocated_msg) ? preallocated_msg : boost::make_shared<sensor_msgs::Image>();
            
            sensor_msgs::Image &new_msg = *new_msg_ptr;
            new_msg.header = cylindrical_history.getHeader();
            new_msg.height = width; // 2*width;
            new_msg.width = width;
            new_msg.encoding = encoding;
            new_msg.is_bigendian = false;
            new_msg.step = width * sizeof(T);
            size_t size = new_msg.step * new_msg.height;
                        
            new_msg.data.resize(size);

            T* data = (T*)new_msg.data.data();
            
            generateFloorImage<T, scale>(cylindrical_history, data, unknown_val, num_threads);
            
            return new_msg_ptr;
        }
        
        
        template <typename T>
        sensor_msgs::ImagePtr generateFloorImageMsg(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr& preallocated_msg);

        
        template <> 
        sensor_msgs::ImagePtr generateFloorImageMsg<float>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
          return generateFloorImageMsg<float, 1>(cylindrical_history, sensor_msgs::image_encodings::TYPE_32FC1, 
                                                dNaN, num_threads, preallocated_msg);
        }
        
        template <>
        sensor_msgs::ImagePtr generateFloorImageMsg<uint16_t>(const utils::ECWrapper& cylindrical_history, int num_threads, sensor_msgs::ImagePtr& preallocated_msg)
        {
          return generateFloorImageMsg<uint16_t, 1000>(cylindrical_history, sensor_msgs::image_encodings::TYPE_16UC1, 
                                                      0, num_threads, preallocated_msg);
        }

        
    }
}

#endif //EGOCYLINDRICAL_FLOOR_DEPTH_IMAGE_CORE_INL_H
