#include <egocylindrical/ecwrapper.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf/LinearMath/Matrix3x3.h>
#include <omp.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>


namespace egocylindrical
{
    
    namespace utils
    {
        /* TODO: ensure that the right size 'ints' are used everywhere. On my current system, the size of int = minimum size of long int, so 32 bit system might fail */
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points)
        {
            const int num_cols = points.getCols();
            
            pcl::PointCloud<pcl::PointXYZ> pcloud;
            
            
            sensor_msgs::PointCloud2::Ptr pcloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
            
            pcl::toROSMsg(pcloud, *pcloud_msg);
            
            pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_cols);
            
            pcloud_msg->width = num_cols;
            pcloud_msg->height = 1;
            
            const float* x = points.getX();
            const float* y = points.getY();
            const float* z = points.getZ();
            
            float* data = (float*) pcloud_msg->data.data();

            
            #pragma omp parallel for num_threads(4)
            for(int j = 0; j < num_cols; ++j)
            {   
                data[4*j] = x[j];
                data[4*j+1] = y[j];
                data[4*j+2] = z[j];
            }

            pcloud_msg->header = points.getHeader();
            
            return pcloud_msg;
        
        }
    }   
}
