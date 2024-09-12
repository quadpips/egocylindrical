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

      ranges[0] = 1;
      ranges[7] = 1.3;
      ranges[13] = 0.7;

    //   test_join(ranges);

      auto iid = InflationIndices<float>(converter.getWidth(), converter.getHScale(), inflation_radius, ranges.data(), inflated.data());
      iid.fillK();
      iid.fillInflated();
      // iid.inflate();
      debugInflate(iid);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_image_inflation_tester");
  test_inflation_indices();
  ros::spin();
}