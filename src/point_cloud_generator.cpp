//
// Created by root on 2/5/18.
//

#include <egocylindrical/point_cloud_generator.h>
#include <egocylindrical/point_cloud_core.h>
#include <egocylindrical/ecwrapper.h>
#include <egocylindrical_msgs/msg/ego_cylinder_points.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>


namespace egocylindrical
{


    EgoCylinderPointCloudGenerator::EgoCylinderPointCloudGenerator(rclcpp::Node::SharedPtr node) :
        node_(node)
    {
        std::cout<<"PointCloud publishing Node Initialized"<<std::endl;
    }
    
    bool EgoCylinderPointCloudGenerator::init()
    {
        ec_sub_.reset();

        // ros::SubscriberStatusCallback info_cb = std::bind(&EgoCylinderPointCloudGenerator::ssCB, this);
        // {
        //     Lock lock(connect_mutex_);
            // pc_pub_ = nh_.advertise<sensor_msgs::msg::PointCloud2>("cylindrical", 2); // , info_cb, info_cb
        pc_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("cylindrical", 2);
        // }
        
        return true;
    }


    void EgoCylinderPointCloudGenerator::ssCB()
    {
        //std::cout << (void*)ec_sub_ << ": " << pc_pub_->get_subscription_count() << std::endl;
        Lock lock(connect_mutex_);
        if (pc_pub_->get_subscription_count() > 0)
        {
            if (ec_sub_) //if currently subscribed... no need to do anything
            {
                
            }
            else
            {
                // ROS_INFO("PointCloud Generator Subscribing");
                // ec_sub_ = nh_.subscribe("egocylindrical_points", 2, &EgoCylinderPointCloudGenerator::ecPointsCB, this);
                ec_sub_ = node_->create_subscription<egocylindrical_msgs::msg::EgoCylinderPoints>("egocylindrical_points", 2, std::bind(&EgoCylinderPointCloudGenerator::ecPointsCB, this, std::placeholders::_1));
            }
      
        }
        else
        {
            ec_sub_.reset();
            // ROS_INFO("PointCloud Generator Unsubscribing");
        }
    }
    
    
    void EgoCylinderPointCloudGenerator::ecPointsCB(const egocylindrical_msgs::msg::EgoCylinderPoints::ConstSharedPtr& ec_msg)
    {
        // ROS_DEBUG("Received EgoCylinderPoints msg");

        if(pc_pub_->get_subscription_count()>0)
        {
          // ros::WallTime// start = ros::WallTime::now();
          
          utils::ECWrapper ec_pts(ec_msg);
          
          sensor_msgs::msg::PointCloud2::ConstSharedPtr msg = utils::generate_point_cloud(ec_pts);
          
          // ROS_DEBUG_STREAM("Generating point cloud took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
          

          // ROS_DEBUG("publish egocylindrical pointcloud");
          
          pc_pub_->publish(*msg);
        }
        
    }




}
