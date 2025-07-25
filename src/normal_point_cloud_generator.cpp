//
// Created by root on 2/5/18.
//

#include <egocylindrical/normal_point_cloud_generator.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>


namespace egocylindrical
{


    NormalPointCloudGenerator::NormalPointCloudGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
        nh_(nh),
        pnh_(pnh)
    {
        std::cout<<"Labeled PointCloud publishing Node Initialized"<<std::endl;
    }
    
    bool NormalPointCloudGenerator::init()
    {
        ec_sub_.shutdown();
        
        ros::SubscriberStatusCallback info_cb = boost::bind(&NormalPointCloudGenerator::ssCB, this);
        {
            Lock lock(connect_mutex_);
            pc_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>("normals", 2, info_cb, info_cb);
        }
        
        return true;
    }


    void NormalPointCloudGenerator::ssCB()
    {
        //std::cout << (void*)ec_sub_ << ": " << pc_pub_.getNumSubscribers() << std::endl;
        Lock lock(connect_mutex_);
        if(pc_pub_.getNumSubscribers()>0)
        {
            if(ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
              // ROS_INFO("Labeled PointCloud Generator Subscribing");
              ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &NormalPointCloudGenerator::ecPointsCB, this);
            }
      
        }
        else
        {
            ec_sub_.shutdown();
            // ROS_INFO("Labeled PointCloud Generator Unsubscribing");
        }
    }
    
    
    void NormalPointCloudGenerator::ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");

        if(pc_pub_.getNumSubscribers()>0)
        {
          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          
          sensor_msgs::msg::PointCloud2::ConstPtr msg = utils::generate_normal_point_cloud(ec_pts);
          
          // ROS_DEBUG_STREAM("Generating labeled point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish egocylindrical labeled pointcloud");
          
          pc_pub_.publish(msg);
        }
        
    }




}
