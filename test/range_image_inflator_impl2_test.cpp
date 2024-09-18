#include <ros/ros.h>
#include <egocylindrical/range_image_inflator_impl2.h>



void test_inflation_indices()
{
      using namespace egocylindrical;

      int width = 64;
      utils::ECParams params;
      params.width = width;
      params.height = 16;
      params.vfov = 2;

      utils::ECConverter converter(params);
      float inflation_radius = 0.5;
      std::vector<float> ranges(width, 1000);
      std::vector<float> inflated(width);

      // ranges[0] = 3;
      // ranges[3] = 2;
      // ranges[6] = 1.5;
      // ranges[7] = 2;  //1
      // ranges[10] = 3;
      // ranges[11] = 0.7;
      // ranges[16] = 0.9;
      // ranges[22] = 1;
      // ranges[30] = 1;

      ranges[0] = 3;
      ranges[3] = 2;
      ranges[6] = 1.5;
      ranges[7] = 1;
      ranges[10] = 3;
      ranges[11] = 0.7;
      ranges[16] = 0.9;
      ranges[22] = 1;
      ranges[30] = 1;

      ranges[37] = 1;
      ranges[40] = 3;
      ranges[41] = 0.7;
      ranges[46] = 0.9;
      ranges[52] = 1;
      ranges[63] = 1;

    //   test_join(ranges);

      auto iid = InflationIndices<float,RowIndexer<float>>(converter.getWidth(), converter.getHScale(), inflation_radius, ranges.data(), inflated.data());
      iid.fillK();
      iid.fillInflated();
      // iid.inflate();
      const bool debug = true;
      debugInflateNew<debug>(iid);
}

void test_vertical_inflation_indices()
{
      using namespace egocylindrical;

      int width = 64;
      int height = 32;
      utils::ECParams params;
      params.width = width;
      params.height = height;
      params.vfov = 2;

      utils::ECConverter converter(params);
      float inflation_height = 0.6;
      std::vector<float> ranges(height, 1000);
      std::vector<float> inflated(height);

      ranges[0] = 3;
      ranges[3] = 2;
      ranges[6] = 1.5;
      ranges[7] = 1;
      ranges[10] = 3;
      ranges[11] = 0.7;
      ranges[16] = 0.9;
      ranges[22] = 1;
      ranges[30] = 1;

    //   test_join(ranges);

      auto iid = InflationIndices<float,ColIndexer<float>>(converter.getHeight(), converter.getVScale(), inflation_height, ranges.data(), inflated.data());
      iid.fillK();
      iid.fillInflated();
      // iid.inflate();
      debugInflate(iid);
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "range_image_inflation_tester");
  test_inflation_indices();
  // test_vertical_inflation_indices();
//   ros::spin();
}