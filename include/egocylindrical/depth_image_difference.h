#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/depth_image_common.h>  //for CleanCameraModel

#include <cv_bridge/cv_bridge.h>    //only need cv::Mat

#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

namespace egocylindrical
{
    namespace utils
    {

        struct DIDiffParams
        {
            float neg_eps, pos_eps;
            bool fill_cloud=false, fill_im=false;
        };

        struct DIDiffResults
        {
            sensor_msgs::PointCloud2::Ptr point_cloud;
            sensor_msgs::Image::Ptr depth_image;
        };

        struct DIDiffRequest
        {
            DIDiffParams params;
            DIDiffResults results;
        };

        template <typename T>
        void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const sensor_msgs::Image& image_msg, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, DIDiffRequest& request);

    }

}

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H