#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/depth_image_common.h>  //for CleanCameraModel

#include <cv_bridge/cv_bridge.h>    //only need cv::Mat

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

namespace egocylindrical
{
    namespace utils
    {

        struct DIDiffParams
        {
            float neg_eps, pos_eps;
            bool fill_cloud=false, fill_im=false, fill_debug=false;
        };

        struct DIDiffDebugging
        {
            sensor_msgs::msg::PointCloud2::Ptr point_cloud, dilated_point_cloud;
            sensor_msgs::msg::Image::SharedPtr depth_image, reproj_depth_image, depth_diff_image, range_image, dilated_range_image;
            visualization_msgs::MarkerArray::Ptr marker_array;
        };

        struct DIDiffResults
        {
            sensor_msgs::msg::PointCloud2::Ptr point_cloud;
            sensor_msgs::msg::Image::SharedPtr depth_image;
            DIDiffDebugging debug;
        };

        struct DIDiffRequest
        {
            DIDiffParams params;
            DIDiffResults results;
        };

        void insertPoints6(const utils::ECWrapper& cylindrical_points, const cv::Mat& image, const sensor_msgs::msg::Image::ConstPtr& image_msg, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, DIDiffRequest& request);

    }

}

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H
