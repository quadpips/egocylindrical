#ifndef EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
#define EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H

#include <egocylindrical/depth_image_remapper.h>
#include <egocylindrical/depth_image_common.h>
#include <egocylindrical/depth_image_difference_debugging_publisher.h>

#include <egocylindrical/ecwrapper.h>

// #include <ros/node_handle.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>



namespace egocylindrical
{
    namespace utils
    {

        class DepthImageInserter
        {
            tf2_ros::Buffer& buffer_;
            std::string fixed_frame_id_;
            ros::NodeHandle pnh_;
            CleanCameraModel cam_model_;
            utils::DepthImageRemapper depth_remapper_;
            ros::Publisher pub_diff_pc_, pub_diff_im_;
            DIDiffDebuggingPublisher debug_pub_;
          
        public:
            DepthImageInserter(tf2_ros::Buffer& buffer, ros::NodeHandle pnh);
            
            bool init();
            bool init(std::string fixed_frame_id);

            bool insert(ECWrapper& cylindrical_points, 
                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                        const sensor_msgs::msg::CameraInfo::ConstPtr& cam_info);

            bool insert(ECWrapper& cylindrical_points, 
                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                        const sensor_msgs::msg::CameraInfo::ConstPtr& cam_info,
                        const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg);

            bool insert(ECWrapper& cylindrical_points, 
                        const sensor_msgs::msg::Image::ConstSharedPtr& image_msg, 
                        const sensor_msgs::msg::CameraInfo::ConstPtr& cam_info,
                        const sensor_msgs::msg::Image::ConstSharedPtr& labels_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& normals_msg);

        };
        
    } //end namespace utils
} //end namespace egocylindrical
#endif //EGOCYLINDRICAL_DEPTH_IMAGE_INSERTER_H
