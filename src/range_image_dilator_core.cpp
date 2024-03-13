#include <egocylindrical/range_image_dilator_core.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <sensor_msgs/Image.h>  //Redundant

#include <stdexcept>    //To throw runtime error



sensor_msgs::Image::Ptr dilateImage(const sensor_msgs::Image::ConstPtr& image_msg_in)
{
    if(image_msg_in->encoding != sensor_msgs::image_encodings::TYPE_16UC1)
    {
//         ROS_ERROR_STREAM("Range Image Dilator only supports encoding '16U'!");
        throw std::runtime_error("Range Image Dilator only supports encoding '16U'!");
    }

    cv_bridge::CvImageConstPtr cv_image_in = cv_bridge::toCvShare(image_msg_in);
//     const cv::Mat image_in = cv_bridge::toCvShare(image_msg_in)->image;
    const cv::Mat image_in = cv_image_in->image;

    cv::Mat image_out(image_in.rows, image_in.cols, image_in.type(), cv::Scalar(0));

    cv::Mat kernel;
    cv::Point anchor(-1,-1);
    int iterations = 1;
    cv::dilate(image_in, image_out, kernel, anchor, iterations);

    cv_bridge::CvImage cv_image_out;
    cv_image_out.header = cv_image_in->header;
    cv_image_out.encoding = cv_image_in->encoding;
    cv_image_out.image = image_out;

    sensor_msgs::Image::Ptr image_msg_out = cv_image_out.toImageMsg();
    return image_msg_out;
}
