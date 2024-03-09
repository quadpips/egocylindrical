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

        template <typename T>
        void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::PointCloud2::Ptr& pcloud_msg);

    }

}

#endif //EGOCYLINDRICAL_DEPTH_IMAGE_DIFFERENCE_H