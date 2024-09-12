#include <cstdint>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/range_image_inflator_impl.h>
#include <queue>

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
    
    
    void convertRange(float in, float& out)
    {
      out = in;
    }
    
    void convertRange(float in, uint16_t& out)
    {
      out = 1000*in;
    }
    

    template<typename T>
    class range_view {

    public:

      range_view(T* data, std::size_t size)
          : m_data ( data ),
            m_size ( size ) { }

      const T* begin() const { return m_data; }
      const T* end() const { return m_data + m_size; }

      T* begin() { return m_data; }
      T* end() { return m_data + m_size; }

    private:

      T* m_data;
      std::size_t m_size;
    };

    // end copied code

    template<typename T>
    range_view<T> get_range_view(T* data, std::size_t size)
    {
      return range_view<T>(data, size);
    }


    // Based from https://stackoverflow.com/a/43313233
    template<typename Iterable, typename Sep>
    class Joiner {
        const std::shared_ptr<Iterable> i_storage_;
        const Iterable& i_;
        const Sep& s_;

    public:
        Joiner(const Iterable& i, const Sep& s) : i_(i), s_(s) {}
        Joiner(const Iterable&& i, const Sep& s) : i_storage_(std::make_shared<Iterable>(i)), i_(*i_storage_), s_(s) {}
        std::string str() const {std::stringstream ss; ss << *this; return ss.str();}
        template<typename I, typename S> friend std::ostream& operator<< (std::ostream& os, const Joiner<I,S>& j);
    };

    template<typename I, typename S>
    std::ostream& operator<< (std::ostream& os, const Joiner<I,S>& j) {
        auto elem = j.i_.begin();
        if (elem != j.i_.end()) {
            os << *elem;
            ++elem;
            while (elem != j.i_.end()) {
                os << j.s_ << *elem;
                ++elem;
            }
        }
        return os;
    }

    template<typename I, typename S>
    inline Joiner<I,S> join(const I& i, const S& s) {return Joiner<I,S>(i, s);}

    template<typename T, typename S, typename C=range_view<T>>
    inline Joiner<C,S> join(T* start, size_t size, const S& s) {return Joiner<C,S>(get_range_view(start, size), s);}

    // End copied code

    template <typename T>
    class InflationIndices
    {
    public:

      size_t width;
      T inflation_radius;
      float scale;

      std::vector<int> K;
      //std::vector<T> R; //TODO: possible replace this with the actual inflated array?
      const T* const ranges;
      T* inflated;

    public:

      InflationIndices(size_t width, float scale, T inflation_radius, const T* const ranges, T* inflated):
        width(width),
        inflation_radius(inflation_radius),
        scale(scale),
        ranges(ranges),
        inflated(inflated)
      {
        K.resize(width, 0);
        // R.resize(width, std::numeric_limits<U>::max());
      }

      void fillK()
      {
        #pragma GCC ivdep
        for(size_t i = 0; i < width; ++i)
        {
          auto range = ranges[i];

          auto k = getNumRowInflationIndices(range, scale, inflation_radius);
          k = std::max(k, 0);
          K[i] = isknown(range) ? k : 0;
        }
      }

      void fillInflated()
      {
        #pragma GCC ivdep
        for(size_t i = 0; i < width; ++i)
        {
          auto range = ranges[i];
          inflated[i] = isknown(range) ? range : std::numeric_limits<T>::max();
        }
      }

      void update(int i, int k, T range, int& k_out, T& range_out)
      {
        auto pr = inflated[i];
        auto pk = K[i];

        if(k > 0 && range <= pr)
        {
          if(k < pk)
          {
            int _k;
            T _range;
            update(i+k, pk-k, pr, _k, _range);
          }
          inflated[i] = range;
          K[i] = k;
          k_out = k;
          range_out = range;
        }
        else
        {
          k_out = pk;
          range_out = pr;
        }
      }

      void inflate()
      {
        int k = 0;
        T range = std::numeric_limits<T>::max();

        for(size_t i = 0; i < width; i++)
        {
          update(i, k, range, k, range);
          --k;
        }

      }

      void printRanges()
      {
        // std::cout << "Ranges: " << ranges << "\n";
        ROS_INFO_STREAM("Ranges: " << join(ranges, width, ','));
      }

      void printInflated()
      {
        // std::cout << "Inflated: " << inflated << "\n";
        ROS_INFO_STREAM("Inflated: " << join(inflated, width, ','));
      }
      
      void printK()
      {
        // std::cout << "K: " << K << "\n";
        ROS_INFO_STREAM("K: " << join(K, ','));
      }
    };

    template <typename T>
    void debugInflate(InflationIndices<T>& iid)
    {
      int k = 0;
      T range = std::numeric_limits<T>::max();

      iid.printRanges();
      iid.printInflated();
      iid.printK();

      for(size_t i = 0; i < iid.width; i++)
      {
        // std::cout << "\ni=" << i << ", k=" << k << ", range=" << range << "\n";
        ROS_INFO_STREAM("i=" << i << ", k=" << k << ", range=" << range);
        iid.update(i, k, range, k, range);
        iid.printInflated();
        iid.printK();
        --k;
      }

    }

    template <typename T>
    void test_join(const std::vector<T>& ranges)
    {
      auto width = ranges.size();
      {
        auto p = ranges.data();
        auto r = range_view<const T>(p, width);

        // std::cout << "\nmanual on range_view: is_const=" << r.is_const() << "\n";
        std::cout << "\nmanual on range_view:\n";
        for(auto v : r)
        {
          std::cout << v << "\n";
        }
      }

      {
        std::cout << "\nmanual on range_view w/ to_string:\n";
        auto r = get_range_view(ranges.data(), width);
        for(auto v : r)
        {
          std::cout << std::to_string(v) << "\n";
        }
      }

      {
        std::cout << "\njoin on vector:\n";
        std::cout << join(ranges, ',') << "\n";
      }

      {
        std::cout << "\njoin on range_view:\n";
        auto r = get_range_view(ranges.data(), width);
        std::cout << join(r, ',') << "\n";
      }

      {
        std::cout << "\njoin on pointer + size:\n";
        auto r = ranges.data();
        std::cout << join(r, width, ',') << "\n";
      }
    }
    
    void test_inflation_indices()
    {
      int width = 64;
      utils::ECParams params;
      params.width = width;
      params.height = 16;
      params.vfov = 2;

      utils::ECConverter converter(params);
      float inflation_radius = 0.1;
      std::vector<float> ranges(width, 1000);
      std::vector<float> inflated(width);

      ranges[0] = 1;
      ranges[7] = 1;
      ranges[13] = 0.5;

      test_join(ranges);

      // {
      //   std::cout << "\nmanual on range_view:\n";
      //   auto r = range_view<float>(ranges.data(), width);
      //   for(auto v : r)
      //   {
      //     std::cout << v << "\n";
      //   }
      // }

      // {
      //   std::cout << "\nmanual on range_view w/ to_string:\n";
      //   auto r = range_view<float>(ranges.data(), width);
      //   for(auto v : r)
      //   {
      //     std::cout << std::to_string(v) << "\n";
      //   }
      // }

      // {
      //   std::cout << "\njoin on vector:\n";
      //   std::cout << join(ranges, ',') << "\n";
      // }

      // {
      //   std::cout << "\njoin on range_view:\n";
      //   auto r = range_view<float>(ranges.data(), width);
      //   std::cout << join(r, ',') << "\n";
      // }

      auto iid = InflationIndices<float>(converter.getWidth(), converter.getHScale(), inflation_radius, ranges.data(), inflated.data());
      iid.fillK();
      iid.fillInflated();
      // iid.inflate();
      debugInflate(iid);
    }



    
    // template<typename T>
    // void inflateRow(const T* ranges, int width, float scale, T inflation_radius, T* inflated)
    // {
    //   InflationIndices<T> inflater(width, scale, inflation_radius);
    //   int cur_k=0;
    //   T cur_range = inflater.R[0];


    //   for(int i = 0; i < width; i++)
    //   {
    //     T range = ranges[i];
    //     int k = inflater.K[i];

    //     if(cur_k > 0)
    //     {
    //       if(!(cur_range > range))
    //       {
    //         range = cur_range;
    //         k = cur_k;
    //       }
    //     }

    //     inflater.update(i, k, range);

        
    //   }

    //     else
          

    //       if(range < cur_range)
    //       {
            

    //       }

    //     }

    //     K[i] = cur_k;

    //       if(start_ind < 0)
    //       {
    //         inflateRowRegion(modified_range, start_ind+width, width, inflated);
    //         inflateRowRegion(modified_range, 0, end_ind, inflated);
    //       }
    //       else if(end_ind >= width)
    //       {
    //         inflateRowRegion(modified_range, start_ind, width, inflated);
    //         inflateRowRegion(modified_range, 0, end_ind-width, inflated);
    //       }
    //       else
    //       {
    //         inflateRowRegion(modified_range, start_ind, end_ind, inflated);
    //       }
    //     }
    //   }
    // }





    
    template<typename T>
    void inflateHorizontally(const T* ranges, int height, int width, float scale, T inflation_radius, int num_threads, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        // inflateRow(ranges+j*width, width, scale, inflation_radius, inflated+j*width);
        auto iid = InflationIndices<T>(width, scale, inflation_radius, ranges+j*width, inflated+j*width);
        iid.fillK();
        iid.fillInflated();
        iid.inflate();
      }

      for(int i = 0; i < height*width; ++i)
      {
        T v = inflated[i];
        inflated[i] = (v == std::numeric_limits<T>::max()) ? std::numeric_limits<T>::quiet_NaN() : v;
      }
      ROS_INFO_STREAM("Inflated");
    }
    
//     template<typename T>
//     void getColInflationIndices(int height, float scale, T inflation_radius, T inflation_height, bool conservative, int row, T range, int& start_ind, int& end_ind)
//     {
//       T row_factor = (row - height/2) * range;
//       T height_factor = inflation_height * scale;
      
//       T top_front_ind = (row_factor - height_factor)/(range + inflation_radius);
//       T top_back_ind = (row_factor - height_factor)/(range - inflation_radius);
//       T bottom_front_ind = (row_factor + height_factor)/(range + inflation_radius);
//       T bottom_back_ind = (row_factor + height_factor)/(range - inflation_radius);
      
//       //int top_ind = (row_factor < height_factor) ? top_front_ind : top_back_ind;
      
//       start_ind = std::min(top_front_ind, top_back_ind) + height/2;
//       end_ind = std::max(bottom_front_ind, bottom_back_ind) + height/2;
//     }
    
    
//     template<typename T>
//     void inflateColumn(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, T* buffer, T* inflated)
//     {
//       for(int j = 0; j < height; j++)
//       {
//         T range = buffer[j*width];
        
//         if(!isknown(range))
//         {
//           continue;
//         }
        
//         bool use_new = false;

//         int inflation_size = getNumColInflationIndices(range, scale, inflation_height);  //Need to think this through, might not be same equation
//         int old_start_ind = std::max(j - inflation_size, 0);
//         int old_end_ind = std::min(j + inflation_size + 1, height);
        
//         int raw_start_ind, raw_end_ind;
// //         getColInflationIndices(height, scale, inflation_radius, inflation_height, conservative, j, range, raw_start_ind, raw_end_ind);
        
//         int start_ind, end_ind;
//         if(use_new)
//         {
//           start_ind = std::max(raw_start_ind, 0);
//           end_ind = std::min(raw_end_ind+1, height);
//         }
//         else
//         {
//           start_ind = old_start_ind;
//           end_ind = old_end_ind;
//         }
        
//         //ROS_DEBUG_STREAM_THROTTLE_NAMED("vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
//         if(old_start_ind != start_ind || old_end_ind != end_ind)
//         {
//           //ROS_DEBUG_STREAM_THROTTLE_NAMED(1, "vertical_inflation.index_changes", "Old start=" << old_start_ind << ", new start=" << start_ind << ", old end=" << old_end_ind << ", new end=" << end_ind);
//         }
        
//         if(range >  inflation_radius)
//         {
//           T modified_range = range - inflation_radius;
        
//           for(int k = start_ind; k < end_ind; k++)
//           {
//             T& val = inflated[k*width];
//             if(!isknown(val) || modified_range < val)
//             {
//               val = modified_range;
//             }
//           }
//         }
//       }
//     }
    
//     template<typename T>
//     void inflateVertically(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
//     {
//       for(int i = 0; i < width; i++)
//       {
//         inflateColumn(height, width, scale, inflation_radius, inflation_height, conservative, buffer+i, inflated+i);
//       }
//     }
    
    template<typename T>
    void inflateRangeImage(const T* ranges, const utils::ECConverter& converter, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    {
      int height = converter.getHeight();
      int width = converter.getWidth();
      
      float hscale = converter.getHScale();
      // float vscale = converter.getVScale();
      
      // inflateHorizontally(ranges, height, width, hscale, inflation_radius, num_threads, buffer);
      inflateHorizontally(ranges, height, width, hscale, inflation_radius, num_threads, inflated);
      // inflateVertically(height, width, vscale, inflation_radius, inflation_height, conservative, num_threads, buffer, inflated);
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


} //end namespace egocylindrical

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "range_image_inflation_tester");
//   egocylindrical::test_inflation_indices();
//   ros::spin();
// }