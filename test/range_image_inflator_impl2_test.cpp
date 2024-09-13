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

    //   ranges[0] = 1;
    //   ranges[7] = 1.3;
    //   ranges[13] = 0.7;
    //   ranges[30] = 0.8;
    //   ranges[31] = 0.9;
    //   ranges[32] = 0.9;
    //   ranges[33] = 0.9;
    //   ranges[34] = 0.9;
    //   ranges[35] = 0.9;
    //   ranges[40] = 1.0;
    //   ranges[45] = 1.3;

    //   ranges[0] = 1;
    //   ranges[1] = 0.9;
    //   ranges[2] = 2;
    //   ranges[3] = 1.3;
    //   ranges[4] = 3;
    //   ranges[5] = 0.7;
    //   ranges[6] = 0.9;
    //   ranges[7] = 1.5;
    //   ranges[8] = 0.9;
    //   ranges[12] = 0.9;
    //   ranges[13] = 2.0;
    //   ranges[15] = 1;

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

      auto iid = InflationIndices<float>(converter.getWidth(), converter.getHScale(), inflation_radius, ranges.data(), inflated.data());
      iid.fillK();
      iid.fillInflated();
      // iid.inflate();
      debugInflate(iid);
}


int main(int argc, char** argv)
{
//   ros::init(argc, argv, "range_image_inflation_tester");
  test_inflation_indices();
//   ros::spin();
}