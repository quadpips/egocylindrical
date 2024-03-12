#include <egocylindrical/depth_image_difference.h>

#include <egocylindrical/ecwrapper.h>   //redundant
#include <egocylindrical/depth_image_common.h>  //redundant
#include <egocylindrical/point_transformer_object.h>

#include <cv_bridge/cv_bridge.h> //redundant
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cstdlib>

#include <tf2/LinearMath/Transform.h>


namespace tf2
{
    //Both functions copied from tf2/src/buffer_core.cpp; ideally, tf2 would declare them in a header so we could just use them directly

    void transformMsgToTF2(const geometry_msgs::Transform& msg, tf2::Transform& tf2)
    {tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

    /** \brief convert Transform to Transform msg*/
    void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::Transform& msg)
    {
        msg.translation.x = tf2.getOrigin().x();
        msg.translation.y = tf2.getOrigin().y();
        msg.translation.z = tf2.getOrigin().z();
        msg.rotation.x = tf2.getRotation().x();
        msg.rotation.y = tf2.getRotation().y();
        msg.rotation.z = tf2.getRotation().z();
        msg.rotation.w = tf2.getRotation().w();
    }
}


namespace egocylindrical
{
    namespace utils
    {
        

        geometry_msgs::TransformStamped getInverse(const geometry_msgs::TransformStamped& transform_msg)
        {
            tf2::Transform transform;
            transformMsgToTF2(transform_msg.transform, transform);
            geometry_msgs::TransformStamped inverted_transform_msg;
            tf2::transformTF2ToMsg(transform.inverse(), inverted_transform_msg.transform);
            inverted_transform_msg.header.stamp = transform_msg.header.stamp;
            inverted_transform_msg.header.frame_id = transform_msg.child_frame_id;
            inverted_transform_msg.child_frame_id = transform_msg.header.frame_id;

            return inverted_transform_msg;
        }

        //immediate insert
        template <typename T>
        void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat& image, const sensor_msgs::Image& image_msg, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, DIDiffRequest& request)
        {
            auto neg_eps = request.params.neg_eps;
            auto pos_eps = request.params.pos_eps;
            
            bool fill_cloud = request.params.fill_cloud;
            bool fill_im = request.params.fill_im;

            sensor_msgs::PointCloud2::Ptr& pcloud_msg = request.results.point_cloud;
            sensor_msgs::Image::Ptr& gen_im_msg = request.results.depth_image;

            cv::Size image_size = cam_model.reducedResolution();
            int image_width = image_size.width;
            int image_height = image_size.height;
            
            const int max_ind = cylindrical_points.getCols();
            float* x = cylindrical_points.getX();
            float* y = cylindrical_points.getY();
            float* z = cylindrical_points.getZ();
            
            PointTransformerObject point_transformer(transform);
            
            const uint scale = DepthScale<T>::scale();

            PointTransformerObject inverse_point_transformer(getInverse(transform));

            // T* gen_im_data;
            // if(fill_im)
            // {
            //     const int num_pixels = image_width * image_height;
            //     gen_im_msg = boost::make_shared<sensor_msgs::Image>();
            //     gen_im_msg->data.resize(sizeof(T) * num_pixels);
            //     gen_im_data = (T*) gen_im_msg->data.data();
            // }

            cv::Mat gen_im_mat;//(image.rows(), image.cols(), image.type());
            if(fill_im)
            {
                gen_im_mat = cv::Mat(image.rows, image.cols, image.type(), dNaN);
            }
            

            float* pc_data;
            if(fill_cloud)
            {
                const int num_pixels = image_width * image_height;
                pcl::PointCloud<pcl::PointXYZ> pcloud;
                pcloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
                pcl::toROSMsg(pcloud, *pcloud_msg);
                pcloud_msg->data.resize(sizeof(pcl::PointXYZ) * num_pixels);
                pcloud_msg->is_dense = true;  //should be false, but seems to work with true
                pc_data = (float*) pcloud_msg->data.data();
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

                            if(fill_im)
                            {
                                auto pp = inverse_point_transformer.transform(prev_point);
                                // gen_im_data[i + j*image_width] = (T)(scale * pp.z);
                                gen_im_mat.at<T>(i,j) = (T)(scale * pp.z);
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
                            //TODO: fill in gen_im_data here
                        }

                        if(save_point)
                        {
                            //TODO: Move this logic to separate class?
                            if(fill_cloud)
                            {
                                pc_data[4*j] = transformed_pnt.x;
                                pc_data[4*j+1] = transformed_pnt.y;
                                pc_data[4*j+2] = transformed_pnt.z;
                                pc_data[4*j+3] = 1;  //Not necessary, only include if improves performance
                            
                                ++pc_counter;
                            }
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

            if(fill_im)
            {
                cv_bridge::CvImage cvim;
                cvim.image = gen_im_mat;
                cvim.header = image_msg.header;
                cvim.encoding = image_msg.encoding;
                gen_im_msg = cvim.toImageMsg();
                // const int num_pixels = image_width * image_height;
                // gen_im_msg = boost::make_shared<sensor_msgs::Image>();
                // gen_im_msg->data.resize(sizeof(T) * num_pixels);
                // gen_im_data = (T*) gen_im_msg->data.data();
            }
        }



        // template <uint16_t>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::PointCloud2::Ptr& pcloud_msg);

        // template <typename float>
        // void insertPoints6(utils::ECWrapper& cylindrical_points, const cv::Mat image, const CleanCameraModel& cam_model, const geometry_msgs::TransformStamped transform, float neg_eps, float pos_eps,  sensor_msgs::PointCloud2::Ptr& pcloud_msg);

    } //end namespace utils
} //end namespace egocylindrical
