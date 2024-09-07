#include <cstdint>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/range_image_inflator_impl.h>

namespace egocylindrical
{    
    bool isknown(uint16_t range)
    {
      return range>0;
    }
    
    bool isknown(float range)
    {
      return range==range;
    }
    
    template<typename T>
    float getInflationAngle(T range, T inflation_radius)
    {
      return std::asin(float(inflation_radius)/range);
    }
    
    template<typename T>
    int getNumRowInflationIndices(T range, float scale, T inflation_radius)  //TODO: construct this using existing functions in ECConverter
    {
      return getInflationAngle(range, inflation_radius) * scale;
    }
    
    template<typename T>
    int getNumColInflationIndices(T range, float scale, T inflation_height)
    {
      return inflation_height * scale / range;
    }
    
    
    template<typename T>
    void inflateRowRegion(T range, int start_ind, int end_ind, T* inflated)
    {
      #pragma GCC ivdep
      for(int ind = start_ind; ind < end_ind; ind++)
      {
        T val = inflated[ind];
        
        //inflated[ind] = (!isknown(val) || range < val) ? range : val;
        
        //T mod_val = isknown(val) ? val : std::numeric_limits<T>::max();
        T mod_val = (val==val) ? val : std::numeric_limits<T>::max();
        inflated[ind] = (range < mod_val) ? range : mod_val;
                
//         if(!isknown(val) || range < val)
//         {
//           val = range;
//         }
        
        
//         if(inflated[ind]< range)
//         {
//         }
//         else
//         {
//           inflated[ind] = range;
//         }
      }
    }
    

    
//     float convertRange(float range)
//     {
//       return range;
//     }
//     
//     float convertRange(uint16_t range)
//     {
//       return float(range)/1000;
//     }
    
    void convertRange(float in, float& out)
    {
      out = in;
    }
    
    void convertRange(float in, uint16_t& out)
    {
      out = 1000*in;
    }
    
    
    
    template<typename T>
    void inflateRow(const T* ranges, int width, float scale, T inflation_radius, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        T range = ranges[i];

        if(!isknown(range))
        {
          continue;
        }
        
        int inflation_size = getNumRowInflationIndices(range, scale, inflation_radius);
        
        int start_ind = i - inflation_size;
        int end_ind = i + inflation_size + 1;
        
        if(range >  inflation_radius)
        {
          T modified_range = range; // - inflation_radius;
          
          if(start_ind < 0)
          {
            inflateRowRegion(modified_range, start_ind+width, width, inflated);
            inflateRowRegion(modified_range, 0, end_ind, inflated);
          }
          else if(end_ind >= width)
          {
            inflateRowRegion(modified_range, start_ind, width, inflated);
            inflateRowRegion(modified_range, 0, end_ind-width, inflated);
          }
          else
          {
            inflateRowRegion(modified_range, start_ind, end_ind, inflated);
          }
        }
      }
    }
    
    template<typename T>
    void inflateHorizontally(const T* ranges, int height, int width, float scale, T inflation_radius, int num_threads, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        inflateRow(ranges+j*width, width, scale, inflation_radius, inflated+j*width);
      }
    }
    
    template<typename T>
    void getColInflationIndices(int height, float scale, T inflation_radius, T inflation_height, bool conservative, int row, T range, int& start_ind, int& end_ind)
    {
      T row_factor = (row - height/2) * range;
      T height_factor = inflation_height * scale;
      
      T top_front_ind = (row_factor - height_factor)/(range + inflation_radius);
      T top_back_ind = (row_factor - height_factor)/(range - inflation_radius);
      T bottom_front_ind = (row_factor + height_factor)/(range + inflation_radius);
      T bottom_back_ind = (row_factor + height_factor)/(range - inflation_radius);
      
      //int top_ind = (row_factor < height_factor) ? top_front_ind : top_back_ind;
      
      start_ind = std::min(top_front_ind, top_back_ind) + height/2;
      end_ind = std::max(bottom_front_ind, bottom_back_ind) + height/2;
    }
    
    
    template<typename T>
    void inflateColumn(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, T* buffer, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        T range = buffer[j*width];
        
        if(!isknown(range))
        {
          continue;
        }
        
        bool use_new = false;

        int inflation_size = getNumColInflationIndices(range, scale, inflation_height);  //Need to think this through, might not be same equation
        int old_start_ind = std::max(j - inflation_size, 0);
        int old_end_ind = std::min(j + inflation_size + 1, height);
        
        int raw_start_ind, raw_end_ind;
//         getColInflationIndices(height, scale, inflation_radius, inflation_height, conservative, j, range, raw_start_ind, raw_end_ind);
        
        int start_ind, end_ind;
        if(use_new)
        {
          start_ind = std::max(raw_start_ind, 0);
          end_ind = std::min(raw_end_ind+1, height);
        }
        else
        {
          start_ind = old_start_ind;
          end_ind = old_end_ind;
        }
        
        //ROS_DEBUG_STREAM_THROTTLE_NAMED("vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
        if(old_start_ind != start_ind || old_end_ind != end_ind)
        {
          //ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
        }
        
        if(range >  inflation_radius)
        {
          T modified_range = range - inflation_radius;
        
          for(int k = start_ind; k < end_ind; k++)
          {
            T& val = inflated[k*width];
            if(!isknown(val) || modified_range < val)
            {
              val = modified_range;
            }
          }
        }
      }
    }
    
    template<typename T>
    void inflateVertically(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    {
      for(int i = 0; i < width; i++)
      {
        inflateColumn(height, width, scale, inflation_radius, inflation_height, conservative, buffer+i, inflated+i);
      }
    }
    
    template<typename T>
    void inflateRangeImage(const T* ranges, const utils::ECConverter& converter, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    {
      int height = converter.getHeight();
      int width = converter.getWidth();
      
      float hscale = converter.getHScale();
      float vscale = converter.getVScale();
      
      inflateHorizontally(ranges, height, width, hscale, inflation_radius, num_threads, buffer);
      inflateVertically(height, width, vscale, inflation_radius, inflation_height, conservative, num_threads, buffer, inflated);
    }
    
    template<typename T>
    void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, bool conservative, int num_threads, sensor_msgs::Image& new_msg, const T unknown_value)
    {
      T converted_inflation_radius;
      convertRange(inflation_radius, converted_inflation_radius);
      
      T converted_inflation_height;
      convertRange(inflation_height, converted_inflation_height);
      
      std::vector<T> buffer(converter.getCols(), unknown_value);
      
      T* inflated_ranges = (T*)new_msg.data.data();
      std::fill(inflated_ranges, inflated_ranges+converter.getCols(), unknown_value);
      
      const T* ranges = (T*)range_msg.data.data();
      
      inflateRangeImage<T>(ranges, converter, converted_inflation_radius, converted_inflation_height, conservative, num_threads, buffer.data(), inflated_ranges);
    }


    void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const float unknown_value)
    {
      inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    }

    void inflateRawRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const uint16_t unknown_value)
    {
      inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    }


} //end namespace