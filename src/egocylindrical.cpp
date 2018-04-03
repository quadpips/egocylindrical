//
// Created by root on 2/5/18.
//

#include "egocylindrical.h"

EgoCylindrical::EgoCylindrical()
{
    image = sensor_msgs::Image();
    cam_info = sensor_msgs::CameraInfo();
    cols = 0;
    rows = 0;
    height = 0;
    width = 0;
}


EgoCylindrical::EgoCylindrical(stixel_estimator::stixelListMsg stixels, sensor_msgs::CameraInfo cam_info) {
    ROS_INFO("start building cylindrical image");
//    this->image = image;
    originalStixel = stixels;
    this->cam_info = cam_info;
//    originImage = cv::Mat(image.height, image.width, CV_32FC1, &image.data[0]).clone();
    height = cam_info.height;
    width = cam_info.width;
    rows = cam_info.height;
    cols = cam_info.width;
    image_geometry::PinholeCameraModel model_t;
    model_t.fromCameraInfo(cam_info);
    double fvp, fhp;
    fvp = fhp = 1 / (tan(2 * M_PI / cols));
    double hc = cols / 2;
    double vc = rows / 2;
    x = y = 0;
    for(auto stixel : stixels.stixels)
    {
        cv::Point2d pt_bottom, pt_top;
        pt_bottom.x = stixel.x;
        pt_bottom.y = stixel.bottom_y;
        pt_top.x = stixel.x;
        pt_top.y = stixel.top_y;


        cv::Point3d Pcyl_top, Pcyl_bottom, Pcyl_t_top, Pcyl_t_bottom;
        pcl::PointXYZ p1, p2;






        Pcyl_top = model_t.projectPixelTo3dRay(pt_top);
        Pcyl_bottom = model_t.projectPixelTo3dRay(pt_bottom);
        Pcyl_top *= stixel.depth;
        Pcyl_bottom *= stixel.depth;

        p1.x = Pcyl_top.x;
        p1.y = Pcyl_top.y;
        p1.z = Pcyl_top.z;

        p2.x = Pcyl_bottom.x;
        p2.y = Pcyl_bottom.y;
        p2.z = Pcyl_bottom.z;

        worldPointCloud.push_back(p1);
        worldPointCloud.push_back(p2);

        Pcyl_t_top = Pcyl_top / cv::sqrt(cv::pow(Pcyl_top.x, 2) + cv::pow(Pcyl_top.z, 2));
        Pcyl_t_bottom = Pcyl_bottom / cv::sqrt(cv::pow(Pcyl_bottom.x, 2) + cv::pow(Pcyl_bottom.z, 2));

        cv::Point2d c_top, c_bottom;
        c_top.x = atan(Pcyl_t_top.x / Pcyl_t_top.z) * fhp + hc;
        c_top.y = Pcyl_t_top.y * fvp + vc;

        c_bottom.x = atan(Pcyl_t_bottom.x / Pcyl_t_bottom.z) * fhp + hc;
        c_bottom.y = Pcyl_t_bottom.y * fvp + vc;


        index.push_back(c_top);
        index.push_back(c_bottom);


        pcl::PointXYZI pointXYZI_top, pointXYZI_bottom;
        pointXYZI_top.x =(float) Pcyl_t_top.x;
        pointXYZI_top.y =(float) Pcyl_t_top.y;
        pointXYZI_top.z = (float) Pcyl_t_top.z;
        pointXYZI_top.intensity = stixel.depth;
        pointXYZI_bottom.x = (float) Pcyl_t_bottom.x;
        pointXYZI_bottom.y = (float) Pcyl_t_bottom.y;
        pointXYZI_bottom.z = (float) Pcyl_t_bottom.z;
        pointXYZI_bottom.intensity = stixel.depth;

        pcloud.push_back(pointXYZI_top);
        pcloud.push_back(pointXYZI_bottom);
        if(Pcyl_t_top.x > x) x = Pcyl_t_top.x;
        if(Pcyl_t_bottom.x > x) x = Pcyl_t_bottom.x;

    }
}


cv::Mat EgoCylindrical::toImage(){
//    int cursor = 0;
    int x, y;
    cv::Mat newImage = cv::Mat::zeros(rows, cols, CV_8UC3);
    for(int i = 0; i < originalStixel.stixels.size() * 2; i++)
    {
        x = (int) index[i].y;
        y = (int) index[i].x;
        if(x >=0 && x < cols && y >=0 && y < rows)
        {
            newImage.at<cv::Vec3b>(x, y)[2] = 0xFF;
        }

    }
    return newImage;
}



