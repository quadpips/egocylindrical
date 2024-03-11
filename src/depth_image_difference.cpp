#include <egocylindrical/depth_image_difference.h>

#include <egocylindrical/ecwrapper.h>   //redundant
#include <egocylindrical/depth_image_common.h>  //redundant
#include <egocylindrical/point_transformer_object.h>

#include <cv_bridge/cv_bridge.h> //redundant
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cstdlib>



namespace egocylindrical
{
    namespace utils
    {

        //immediate insert
        template <typename T>
        void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, DIDiffRequest& request)
        {
            auto neg_eps = request.params.neg_eps;
            auto pos_eps = request.params.pos_eps;
            
            bool fill_cloud = request.params.fill_cloud;

            sensor_msgs::PointCloud2::Ptr& pcloud_msg = request.results.point_cloud;

            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();


            float* data;
            if(fill_cloud)
            {
                const int num_pixels = image_width * image_height;
                pcl::PointCloud<pcl::PointXYZ> pcloud;
                pcloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
                pcl::toROSMsg(pcloud, *pcloud_msg);
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_pixels);
                pcloud_msg->is_dense = true;  //should be false, but seems to work with true
                data = (float*) pcloud_msg->data.data();
            }

            int pc_counter = 0;

            for(int i = 0; i < image_height; ++i)
            {
                for(int j = 0; j < image_width; ++j)
                {                       
                    cv::Point2d pt;
                    pt.x = j;
                    pt.y = i;
                    
                    T depth = image.at<T>(i,j);
                    
                    
                    // if(depth>0)  //Only insert actual points (works for both float and uint16)
                    {                        
                        cv::Point3f ray = cam_model.projectPixelTo3dRay(pt);
                        cv::Point3f world_pnt = ray * (((float) depth)/scale);
                        cv::Point3f transformed_pnt = point_transformer.transform(world_pnt);
                        
                        int cyl_idx = cylindrical_points.worldToCylindricalIdx(transformed_pnt);
                        bool save_point = false;
                        
                        if(cyl_idx >= 0  && cyl_idx < max_ind)
                        {
                            float range_sq = worldToRangeSquared(transformed_pnt);
                    
                            cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                            
                            float prev_range_sq = worldToRangeSquared(prev_point);
                            
                            float range = std::sqrt(range_sq);
                            float prev_range = std::sqrt(prev_range_sq);
                            
                            bool vr = !std::isnan(range);
                            bool vpr = !std::isnan(prev_range);

                            if(vr && vpr)
                            {
                                float diff = range - prev_range;
                                if(diff > pos_eps || diff < neg_eps)
                                {
                                    save_point = true;
                                }
                            }
                            else if(vr)
                            {
                                save_point = true;
                            }
                            else //if(vpr)
                            {
                                //save_point = false;
                            }

                            if(!(prev_range_sq <= range_sq)) //overwrite || 
                            {   
                                x[cyl_idx] = transformed_pnt.x;
                                y[cyl_idx] = transformed_pnt.y;
                                z[cyl_idx] = transformed_pnt.z;
                            }
                        }
                        else
                        {
                            cyl_idx = cylindrical_points.worldToCanIdx(transformed_pnt);
                        
                            if(cyl_idx >=0 && cyl_idx < cylindrical_points.getNumPts())
                            {
                                float can_depth = worldToCanDepth(transformed_pnt);
                                
                                cv::Point3f prev_point(x[cyl_idx], y[cyl_idx], z[cyl_idx]);
                                
                                float prev_can_depth = worldToCanDepth(prev_point);
                                
                                bool vd = !std::isnan(can_depth);
                                bool vpd = !std::isnan(prev_can_depth);

                                if(vd && vpd)
                                {
                                    float diff = can_depth - prev_can_depth;
                                    if(diff > pos_eps || diff < neg_eps)
                                    {
                                        save_point = true;
                                    }
                                }
                                else if(vd)
                                {
                                    save_point = true;
                                }
                                else //if(vpr)
                                {
                                    //save_point = false;
                                }

                                if(!(prev_can_depth <= can_depth)) //overwrite || 
                                {   
                                    
                                    x[cyl_idx] = transformed_pnt.x;
                                    y[cyl_idx] = transformed_pnt.y;
                                    z[cyl_idx] = transformed_pnt.z;
                                }
                            }
                        }

                        if(save_point)
                        {
                            data[4*j] = transformed_pnt.x;
                            data[4*j+1] = transformed_pnt.y;
                            data[4*j+2] = transformed_pnt.z;
                            data[4*j+3] = 1;  //Not necessary, only include if improves performance
                        
                            ++pc_counter;
                        }
                    }
                    
                }
            }

            if(fill_cloud)
            {
                //TODO: if no points added to point cloud, don't bother publishing
                pcloud_msg->width = pc_counter;
                pcloud_msg->height = 1;
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ)*pc_counter);
                pcloud_msg->row_step = static_cast<uint32_t> (sizeof (pcl::PointXYZ) * pcloud_msg->width);
            }
        }



        // template <uint16_t>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::PointCloud2::Ptr& pcloud_msg);

        // template <typename float>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::PointCloud2::Ptr& pcloud_msg);

    } //end namespace utils
} //end namespace egocylindrical
