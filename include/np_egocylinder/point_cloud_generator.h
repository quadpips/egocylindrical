//
// Created by root on 2/5/18.
//

#include <np_egocylinder/ecwrapper.h>
#include <np_egocylinder/EgoCylinderPoints.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>


namespace np_egocylinder
{

    namespace utils
    {
        sensor_msgs::PointCloud2::ConstPtr generate_point_cloud(const utils::ECWrapper& points);
    }

class EgoCylinderPointCloudGenerator
{
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pc_pub_;
    ros::Subscriber ec_sub_;

    
    
public:

    EgoCylinderPointCloudGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);

    bool init();


private:
    
    void ssCB();

    void ecPointsCB(const np_egocylinder::EgoCylinderPoints::ConstPtr& ec_msg);

};


}
