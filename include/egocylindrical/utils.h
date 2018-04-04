#ifndef EGOCYLINDRICAL_UTILS_H
#define EGOCYLINDRICAL_UTILS_H

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/ecstixel.h>

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <omp.h>
#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/EgoCylinderPoints.h>

#include <stixel_estimator/stixelListMsg.h>
#include <stixel_estimator/stixelMsg.h>

#include <drawing/gil/colors.hpp>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
namespace egocylindrical
{

namespace utils
{
    
    

    inline
    float worldToRange(cv::Point3f point)
    {
        return std::sqrt(point.x*point.x + point.z*point.z);
    }
   /*
    
    cv::Point3f worldToCylindrical(cv::Point3f point, int cyl_width, int cyl_height, double hfov, double vfov)
    {
        cv::Point3f Pcyl_t = point / cv::sqrt(cv::pow(point.x, 2) + cv::pow(point.z, 2));   
        
        double theta = std::atan2(point.x,point.z);
        double phi = std::atan2(point.y,point.z);
        
        cyl_width* theta /hfov
        
        
    }
    */
    
    /*
    inline
    cv::Point cylindricalToImage(cv::Point3f point)
    {
        
    }
    */
    
    inline
    cv::Point3f projectWorldToCylinder(const cv::Point3f& point)
    {
        cv::Point3f Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
        return Pcyl_t;
    }
    

    inline
    cv::Point worldToCylindricalImage(const cv::Point3f& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
    {
        
        cv::Point3f p_cyl = projectWorldToCylinder(point);
        
        
        float x = std::atan2(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
        float y = p_cyl.y * v_scale + cyl_height / 2;
                
        cv::Point im_pt(x,y);
        return im_pt;
    }

    
    struct CylindricalCoordsConverter
    {
       int width, height;
       float hfov, vfov;
       float h_scale, v_scale;
       float h_offset=0, v_offset=0;
       
       CylindricalCoordsConverter()
       {
       }
       
       CylindricalCoordsConverter(int width, int height, float hfov, float vfov):
            width(width),
            height(height),
            hfov(hfov),
            vfov(vfov)
        {
            h_scale = width/hfov;
            v_scale = height/vfov;
        }
        
        inline
        cv::Point worldToCylindricalImage(cv::Point3f point) const
        {
            return utils::worldToCylindricalImage(point, width, height, h_scale, v_scale, h_offset, v_offset);
        }
        
        inline
        cv::Rect getImageROI() const
        {
            return cv::Rect(cv::Point(), cv::Size(width,height));
            
        }
        
        inline
        int getStixelRange() const
        {
            return width;
        }
        
        /*
        inline
        cv::Point3f getWorldPoint(const cv::Mat& image, const cv::Point& image_pnt) const
        {
            
        }
        */
        
        /* TODO: check ROI and perform worldToCylindricalImage call in here
         * Goal: abstract away the underlying data representation from the rest of my code
         */
        
        inline
        bool setWorldPoint(cv::Mat& image, const cv::Point3f& world_pnt, const cv::Point& image_pnt)
        {
            image.at<float>(0,image_pnt.y*width + image_pnt.x) = world_pnt.x;
            image.at<float>(1,image_pnt.y*width + image_pnt.x) = world_pnt.y;
            image.at<float>(2,image_pnt.y*width + image_pnt.x) = world_pnt.z;
            
            return true;
        }
        
        
        
    };
    
    
    inline
    void addPoints(utils::ECStixel& cylindrical_history, utils::ECStixel& new_points, const CylindricalCoordsConverter& ccc, bool overwrite)
    {
        const cv::Rect image_roi = ccc.getImageROI();
        
        std::vector<float>* x = cylindrical_history.getX();
        std::vector<float>* by = cylindrical_history.getBY();
        std::vector<float>* ty = cylindrical_history.getTY();
        std::vector<float>* z = cylindrical_history.getZ();
        std::vector<stixel_estimator::stixelMsg>* stixel = cylindrical_history.getStixels();
        
        const std::vector<float>* n_x = new_points.getX();
        const std::vector<float>* n_by = new_points.getBY();
        const std::vector<float>* n_ty = new_points.getTY();
        const std::vector<float>* n_z = new_points.getZ();
        const std::vector<stixel_estimator::stixelMsg>* new_stixel = new_points.getStixels();
        

        ROS_DEBUG("Relocated the propagated image");
        //#pragma omp parallel for
        for(int i = 0; i < new_points.getSize(); ++i)
        {
            
//             cv::Point3f world_pnt_b(n_x->at(i),n_by->at(i),n_z->at(i));
            cv::Point3f world_pnt_b(n_x->at(i),0,n_z->at(i));
            
            float depth = worldToRange(world_pnt_b);
            
            if(depth==depth)
            {
                // The following 3 steps could probably be moved to the point propagation step and performed in parallel
                // It will depend on whether the extra memory access for the steps cost more or less than the calculations
                cv::Point image_pnt = ccc.worldToCylindricalImage(world_pnt_b);
                
                int idx =  image_pnt.x;
                

                if(image_roi.contains(image_pnt))
                {
//                     cv::Point3f prev_point(x->at(idx), by->at(idx), z->at(idx));
                    cv::Point3f prev_point(x->at(idx), 0, z->at(idx));
                    
                    float prev_depth = worldToRange(prev_point);

                    if(!(prev_depth >= depth)) //overwrite || 
                    {
                        
                        x->at(idx) = world_pnt_b.x;
                        by->at(idx) = world_pnt_b.y;
                        z->at(idx) = world_pnt_b.z;
                        stixel->at(idx) = new_stixel->at(i);
                    }
                }
                else
                {
                    //ROS_DEBUG_STREAM("Outside of image!: (" << world_pnt << " => " << image_pnt);
                }
            }
        }
    
        
        
    }
    


    
    inline
    void addStixel(utils::ECStixel& cylindrical_history, const std::vector<stixel_estimator::stixelMsg>& stixels, const CylindricalCoordsConverter& ccc, const image_geometry::PinholeCameraModel& cam_model)
    {
        ROS_DEBUG("Generating depth to cylindrical image mapping");
        
        const cv::Rect image_roi = ccc.getImageROI();
        const int width = ccc.width;
        //const int height = ccc.height;
        
        std::vector<float>* x = cylindrical_history.getX();
        std::vector<float>* by = cylindrical_history.getBY();
        std::vector<float>* ty = cylindrical_history.getTY();
        std::vector<float>* z = cylindrical_history.getZ();
        std::vector<stixel_estimator::stixelMsg>* stixel_history = cylindrical_history.getStixels();
        
        for(int i = 39; i < stixels.size(); ++i)
        {
            if(stixels[i].disparity > 5)
            {
                cv::Point2d pt_b;
                pt_b.x = stixels[i].x;
                pt_b.y = stixels[i].bottom_y;
                
                float depth = stixels[i].depth;
                
                if(depth==depth)
                {
                    cv::Point3f world_pnt_b = cam_model.projectPixelTo3dRay(pt_b)*depth;
                    cv::Point image_pnt_b = ccc.worldToCylindricalImage(world_pnt_b);
                    int idx = image_pnt_b.x;
                    
                    if(image_roi.contains(image_pnt_b))
                    {
                        x->at(idx) = world_pnt_b.x;
                        by->at(idx) = world_pnt_b.y;
                        z->at(idx) = world_pnt_b.z;
                        stixel_history->at(idx) = stixels[i];
                    }
                }
            }
        }
        
    }
    
    
    inline
    void addStixel(utils::ECStixel& cylindrical_history, const stixel_estimator::stixelListMsg::ConstPtr& stixels, const CylindricalCoordsConverter& ccc, const image_geometry::PinholeCameraModel& cam_model)
    {
        const std::vector<stixel_estimator::stixelMsg> stixel = stixels->stixels;
        addStixel(cylindrical_history, stixel, ccc, cam_model);
    }
    
    
    
    // Functions defined in separate compilation units:
    sensor_msgs::ImagePtr getRawRangeImageMsg(const utils::ECWrapper& cylindrical_history);
    
    void transformPoints(utils::ECStixel& points, const geometry_msgs::TransformStamped& trans);
    
    //sensor_msgs::PointCloud2 generate_point_cloud(const utils::ECWrapper& points);
    
    inline
    void HSV2RGB(float h_, float s_, float v_, std_msgs::ColorRGBA& color)
    {
        float h = h_ *   2.0f; // 0-360
        float s = s_; // 0.0-1.0
        float v = v_; // 0.0-1.0

        float r, g, b; // 0.0-1.0

        int   hi = (int)(h / 60.0f) % 6;
        float f  = (h / 60.0f) - hi;
        float p  = v * (1.0f - s);
        float q  = v * (1.0f - s * f);
        float t  = v * (1.0f - s * (1.0f - f));

        switch(hi) {
            case 0: r = v, g = t, b = p; break;
            case 1: r = q, g = v, b = p; break;
            case 2: r = p, g = v, b = t; break;
            case 3: r = p, g = q, b = v; break;
            case 4: r = t, g = p, b = v; break;
            case 5: r = v, g = p, b = q; break;
        }

        color.r = b; // dst_r : 0-255
        color.g = g; // dst_r : 0-255
        color.b = r; // dst_r : 0-255
        color.a = 1.0;
    }
    
    inline
    void stixels_to_rviz(const std::vector<stixel_estimator::stixelMsg>& stixels, image_geometry::PinholeCameraModel p_c_model, float baseline, visualization_msgs::Marker& marker)
    {
        marker.header.frame_id = "stereo_camera_optical_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "stixel_representation";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.05;
        marker.scale.z = 0.05;

        float saturation = 1.0;
        float value = 1.0;
        for (auto it = stixels.begin(); it != stixels.end(); it++) 
        {
            stixel_estimator::stixelMsg t_stixel = *it;
            if (t_stixel.types == "\1") continue;
            int u = t_stixel.x;
            
            int v = t_stixel.bottom_y;
    //         int disparity = t_stixel.disparity;
    //         float z = baseline * p_c_model.fx() / disparity;
            float z = t_stixel.depth;
            //cout << z << endl;
            if (!std::isfinite(z)) continue;
            cv::Point3d cartesian_coord = p_c_model.projectPixelTo3dRay(cv::Point2d(u, v));
            cartesian_coord = cartesian_coord * z;
            //cout << cartesian_coord << endl;
            geometry_msgs::Point p;
            p.x = cartesian_coord.x;
            p.y = cartesian_coord.y;
            p.z = cartesian_coord.z;
            if (p.z > 7.0) continue;

            std_msgs::ColorRGBA color;
            float hue = 0.05 * p.z;
            if (hue > 0.5) hue = 0.5;
            if (hue < 0) hue = 0;
            hue = hue * 180;
            HSV2RGB(hue, saturation, value, color);
            

            marker.points.push_back(p);
            marker.colors.push_back(color);
            p.y -= 2;
            marker.points.push_back(p);
            marker.colors.push_back(color);
        
        }
    }
    
    inline
    void stixels_to_rviz(utils::ECStixel& stixels, visualization_msgs::Marker& marker)
    {
        marker.header.frame_id = "stereo_camera_optical_frame";
        marker.header.stamp = ros::Time::now();
        marker.ns = "stixel_representation";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = 2;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.05;
        marker.scale.z = 0.05;

        float saturation = 1.0;
        float value = 1.0;
        for (int it = 0; it < stixels.getSize(); it++) 
        {
            const std::vector<float>* x = stixels.getX();
            const std::vector<float>* by = stixels.getBY();
            const std::vector<float>* z = stixels.getZ();
            const std::vector<stixel_estimator::stixelMsg>* new_stixel = stixels.getStixels();
            
            //cout << z << endl;
             if (new_stixel->at(it).types == "\1") continue;
            if (std::isnan(z->at(it))) continue;
            cv::Point3d cartesian_coord(x->at(it), by->at(it), z->at(it));
            //cout << cartesian_coord << endl;
            geometry_msgs::Point p;
            p.x = cartesian_coord.x;
            p.y = cartesian_coord.y;
            p.z = cartesian_coord.z;
            if (p.z > 7.0) continue;

            std_msgs::ColorRGBA color;
            float hue = 0.05 * p.z;
            if (hue > 0.5) hue = 0.5;
            if (hue < 0) hue = 0;
            hue = hue * 180;
            HSV2RGB(hue, saturation, value, color);
            

            marker.points.push_back(p);
            marker.colors.push_back(color);
            p.y -= 2;
            marker.points.push_back(p);
            marker.colors.push_back(color);
        
        }
    }

}

}

#endif
