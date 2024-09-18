#ifndef EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_IMPL2_H
#define EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_IMPL2_H

#include <cstdint>
#include <iomanip>
#include <egocylindrical/ecwrapper.h>
// #include <queue>
#include <sensor_msgs/Image.h>


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

    struct JoinerFormat
    {
      int width;
      int precision;

      JoinerFormat(int width, int precision):
        width(width),
        precision(precision) {}
    };

    // Based on https://stackoverflow.com/a/43313233
    template<typename Iterable, typename Sep>
    class Joiner {
        const std::shared_ptr<Iterable> i_storage_;
        const Iterable& i_;
        const Sep& s_;
        const JoinerFormat fmt;


    public:
        Joiner(const Iterable& i, const Sep& s, JoinerFormat fmt) : i_(i), s_(s), fmt(fmt) {}
        Joiner(const Iterable&& i, const Sep& s, JoinerFormat fmt) : i_storage_(std::make_shared<Iterable>(i)), i_(*i_storage_), s_(s), fmt(fmt) {}
        std::string str() const {std::stringstream ss; ss << *this; return ss.str();}
        template<typename I, typename S> friend std::ostream& operator<< (std::ostream& os, const Joiner<I,S>& j);
    };

    template<typename I, typename S>
    std::ostream& operator<< (std::ostream& os, const Joiner<I,S>& j) {
        auto elem = j.i_.begin();
        if (elem != j.i_.end()) {
            if(j.fmt.width > 0)
            {
              os << std::setw(j.fmt.width) << *elem;
              ++elem;
              while (elem != j.i_.end()) {
                  os << j.s_ << std::setw(j.fmt.width) << *elem;
                  ++elem;
              }
            }
            else
            {
              os << *elem;
              ++elem;
              while (elem != j.i_.end()) {
                  os << j.s_ << *elem;
                  ++elem;
              }
            }
        }
        return os;
    }

    template<typename I, typename S>
    inline Joiner<I,S> join(const I& i, const S& s, int width=-1) {return Joiner<I,S>(i, s, JoinerFormat(width,-1));}

    template<typename T, typename S, typename C=range_view<T>>
    inline Joiner<C,S> join(T* start, size_t size, const S& s, int width=-1) {return Joiner<C,S>(get_range_view(start, size), s, JoinerFormat(width,-1));}

    // End copied code



    template<typename T, typename I>
    class IndexConverter
    {
      float scale;
      T inflation_magnitude;

    public:
      IndexConverter(float scale, T inflation_magnitude): scale(scale), inflation_magnitude(inflation_magnitude){}
      int operator() (T range) { return I::impl(range, scale, inflation_magnitude);}
    };

    template<typename T>
    class RowIndexer
    {
    public:
      static
      int impl(T range, float scale, T inflation_radius) { return getNumRowInflationIndices(range, scale, inflation_radius); }
    };

    template<typename T>
    class ColIndexer
    {
    public:
      static
      int impl(T range, float scale, T inflation_height) { return getNumColInflationIndices(range, scale, inflation_height); }
    };



    template <typename T, typename I, bool Wraparound>
    class InflationIndices
    {
    public:

      int width;
      T inflation_radius;
      float scale;

      std::vector<int> K, Kref; //Kref would be a good candidate for reusing an existing buffer
      const T* const ranges;
      T* inflated;
      int row;

      IndexConverter<T,I> indexer;

    public:

      InflationIndices(int width, float scale, T inflation_radius, const T* const ranges, T* inflated, int row=-1):
        width(width),
        inflation_radius(inflation_radius),
        scale(scale),
        ranges(ranges),
        inflated(inflated),
        row(row),
        indexer(scale, inflation_radius)
      {
        Kref.resize(width, 0);
        // R.resize(width, std::numeric_limits<U>::max());
      }

      void fillK()
      {
        #pragma GCC ivdep
        for(int i = 0; i < width; ++i)
        {
          auto range = ranges[i];

          auto k = indexer(range); //getNumRowInflationIndices(range, scale, inflation_radius);
          auto k2 = std::max(k, 0);
          auto k3 = isknown(range) ? k2 : 0;
          Kref[i] = k3;
        }
      }

      void fillInflated()
      {
        #pragma GCC ivdep
        for(int i = 0; i < width; ++i)
        {
          auto range = ranges[i];
          inflated[i] = isknown(range) ? range : std::numeric_limits<T>::max();
        }
      }

      int getIndex(int i)
      {
        if(Wraparound)
        {
          return (i + width) % width;
        }
        else
        {
          return i;
        }
      }

    protected:
      // TODO: Return bool indicating whether this update had any effects, will need something like that to know when to stop inflation when wraparound enabled
      template<int dir>
      void update_future(int i, int k, T r)
      {
        i = getIndex(i);
        if(k<=0 || i >= width || i < 0) //Temporary, will address wraparound later
        {
          return;
        }
        //TODO: modulo i by width to transparently handle wrap around
        auto pr = inflated[i];
        auto pk = K[i];

        if(r <= pr)
        {
          if(k < pk)
          {
            update_future<dir>(i + dir*k, pk-k, pr);
          }
          inflated[i] = r;
          K[i] = k;
        }
        else
        {
          if(pk > 0 && pk < k)
          {
            update_future<dir>(i + dir*pk, k-pk, r);
          }
        }
      }

    public:
      template<int dir>
      void update(int i, int k, T range, int& k_out, T& range_out)
      {
        update_future<dir>(i, k, range);
        k_out = K[i];
        range_out = inflated[i];
      }

      void resetK()
      {
        K = Kref;
      }

      void inflate()
      {
        resetK();
        int k = 0;
        T range = std::numeric_limits<T>::max();

        for(int i = 0; i < width; ++i)
        {
          update<1>(i, k, range, k, range);
          --k;
        }
        //NOTE: if k > 0, need to wrap around to the front until k==0
        //Only then can the reverse pass commence. 
        resetK(); //Reinitialize K with original entries
        for(size_t ii = width; ii > 0; --ii)
        {
          auto i = ii - 1;
          update<-1>(i, k, range, k, range);
          --k;
        }
        //Similarly, may need to wrap around to back
      }

      // void inflate2()
      // {
      //   resetK();
      //   const int start_ind = 0;
      //   int ind = start_ind + 1
      //   while(ind != start_ind)
      //   {
      //     update_future<1>(i);
      //     ind = getIndex(ind + 1);
      //   }
      // }

      // template<int dir>
      // void inflationPass()
      // {
      //   const int start_ind = 0;
      //   int i = start_ind + dir;

      //   resetK();
      //   while(i != start_ind)
      //   {
      //     i = getIndex(i);
      //     update_future<1>(i);
      //     i += dir;
      //   }

      //   while(true)
      //   {
      //     i = getIndex(i);
      //     if(!update_future<1>(i))
      //     {
      //       break;
      //     }
      //     i += dir;
      //   }
      // }

      void printRanges()
      {
        std::cout << "Ranges:  " << join(ranges, width, ',', 4) << "\n";
        // ROS_INFO_STREAM("Ranges:  " << join(ranges, width, ',', 4));
      }

      void printInflated()
      {
        std::cout << "Inflated:" << join(inflated, width, ',', 4) << "\n";
        // ROS_INFO_STREAM("Inflated:" << join(inflated, width, ',', 4));
      }
      
      void printK()
      {
        std::cout << "K:       " << join(K, ',', 4) << "\n";
        // ROS_INFO_STREAM("K:       " << join(K, ',', 4));
      }
    };

    template <typename T, typename I, bool W>
    void debugInflate(InflationIndices<T,I,W>& iid)
    {
      int k = 0;
      T range = std::numeric_limits<T>::max();

      iid.resetK();

      iid.printRanges();
      iid.printInflated();
      iid.printK();

      for(int i = 0; i < iid.width; i++)
      {
        // std::cout << "\ni=" << i << ", k=" << k << ", range=" << range << "\n";
        std::cout << "\n";
        // ROS_INFO_STREAM("i=" << i << ", k=" << k << ", range=" << range);
        // iid.update(i, k, range, k, range, true);
        iid.template update<1>(i, k, range, k, range);
        iid.printRanges();
        iid.printInflated();
        iid.printK();
        --k;
      }
      // for(size_t ii = iid.width; ii > 0; --ii)
      // {
      //   auto i = ii - 1;
      //   // std::cout << "\ni=" << i << ", k=" << k << ", range=" << range << "\n";
      //   ROS_INFO_STREAM("i=" << i << ", k=" << k << ", range=" << range);
      //   iid.update(i, k, range, k, range);
      //   iid.printInflated();
      //   iid.printK();
      //   --k;
      // }

    }


    // void AlternativeLoop(int dir)
    // {
    //   auto p_iter = ... //this can probably be a const iterator since represents the values of the previously updated cell
    //   auto iter = p_iter + 1;

    //   while(iter != start_iter)
    //   {
    //     auto p = *p_iter;
    //     auto pk = p.k;  //iterator holds pointers to k and r and returns references to the currently pointed indices of k and r
    //     auto pr = p.r;

    //     update(*iter)

    //     iter += dir;
    //   }

    //   bool done = false;
    //   while(!done)
    //   {
    //     auto p = *p_iter;
    //     auto pk = p.k;  //iterator holds pointers to k and r and returns references to the currently pointed indices of k and r
    //     auto pr = p.r;

    //     update(*iter)

    //     iter += dir;
    //   }

    // }


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
    

    template<typename T, typename I, bool W>
    void inflateAxis(const T* ranges, int height, int width, float scale, T inflation_radius, int num_threads, T* inflated)
    {
      for(int j = 0; j < height; j++)
      {
        // inflateRow(ranges+j*width, width, scale, inflation_radius, inflated+j*width);
        auto iid = InflationIndices<T,I,W>(width, scale, inflation_radius, ranges+j*width, inflated+j*width, j);
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
    
    template<typename T>
    void inflateRange(const T* ranges, int height, int width, T inflation_radius, int num_threads, T* inflated)
    {
      for(int i = 0; i < height*width; ++i)
      {
        inflated[i] = ranges[i] - inflation_radius;
      }
    }

    // template<typename T>
    // void inflateHorizontally(const T* ranges, int height, int width, float scale, T inflation_radius, int num_threads, T* inflated)
    // {
    //   inflateAxis<T,RowIndexer>(ranges, height, width, scale, inflation_radius, num_threads, inflated);
    // }

    //Based on https://stackoverflow.com/a/59132848
    template<typename T>
    void transpose(const T* in, int m, int n, T* out)
    {
      // cv::Mat mat(height, width, CV_32FC1, in);
      
      // cv::transpose(m)
      for (int i = 1; i <= n; i++)
      {
          for (int j = 1; j <= m; j++)
          {
              out[(j - 1) * n + i - 1] = in[(i - 1) * m + j - 1];
          }
      }
    }

    // template<typename T>
    // void inflateVertically(int height, int width, float scale, T inflation_radius, T inflation_height, bool conservative, int num_threads, T* buffer, T* inflated)
    // {

    // }

    
    template<typename T>
    void inflateRangeImage(const T* ranges, const utils::ECConverter& converter, T inflation_radius, T inflation_height, float vertical_offset, int num_threads, T* buffer, T* inflated)
    {
      int height = converter.getHeight();
      int width = converter.getWidth();
      
      float hscale = converter.getHScale();
      float vscale = converter.getVScale();

      std::vector<T> buffer2(width*height);
      std::vector<T> buffer3(width*height);
      std::vector<T> buffer4(width*height);

      auto deltaT = [](const std::string& name, const ros::WallTime& start, const ros::WallTime& end)
      {
        std::stringstream s;
        s << name << " time: " << (end-start).toSec()*1000 << "ms; ";
        return s.str();
      };
      
      
      {
        ros::WallTime start = ros::WallTime::now();
        inflateAxis<T,RowIndexer<T>,true>(ranges, height, width, hscale, inflation_radius, num_threads, buffer);
        ros::WallTime rows_done = ros::WallTime::now();
        transpose(buffer, width, height, buffer2.data());
        ros::WallTime transpose1 = ros::WallTime::now();
        inflateAxis<T,ColIndexer<T>,false>(buffer2.data(), width, height, vscale, inflation_height, num_threads, buffer3.data());
        ros::WallTime cols_done= ros::WallTime::now();
        transpose(buffer3.data(), height, width, buffer4.data());
        ros::WallTime transpose2 = ros::WallTime::now();
        inflateRange<T>(buffer4.data(), height, width, inflation_radius, num_threads, inflated);
        ros::WallTime done = ros::WallTime::now();

        ROS_INFO_STREAM_NAMED("timing", deltaT("row inflation", start, rows_done) << deltaT("transpose1", rows_done, transpose1) << deltaT("col inflation", transpose1, cols_done) <<
            deltaT("transpose2", cols_done, transpose2) << deltaT("range", transpose2, done));

      }


      // inflateVertically(height, width, vscale, inflation_radius, inflation_height, conservative, num_threads, buffer, inflated);
    }
    
    template<typename T>
    void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, float vertical_offset, int num_threads, sensor_msgs::Image& new_msg, const T unknown_value)
    {
      T converted_inflation_radius;
      convertRange(inflation_radius, converted_inflation_radius);
      
      T converted_inflation_height;
      convertRange(inflation_height, converted_inflation_height);
      
      std::vector<T> buffer(converter.getCols(), unknown_value);
      
      T* inflated_ranges = (T*)new_msg.data.data();
      std::fill(inflated_ranges, inflated_ranges+converter.getCols(), unknown_value);
      
      const T* ranges = (T*)range_msg.data.data();
      
      inflateRangeImage<T>(ranges, converter, converted_inflation_radius, converted_inflation_height, vertical_offset, num_threads, buffer.data(), inflated_ranges);
    }


    // void inflateRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const float unknown_value)
    // {
    //   inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    // }

    // void inflateRawRangeImage(const sensor_msgs::Image& range_msg, const utils::ECConverter& converter, float inflation_radius, float inflation_height, int num_threads, sensor_msgs::Image& new_msg, const uint16_t unknown_value)
    // {
    //   inflateRangeImage(range_msg, converter, inflation_radius, inflation_height, false, num_threads, new_msg, unknown_value);
    // }


} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_RANGE_IMAGE_INFLATOR_IMPL2_H