#include <egocylindrical/floor_image_generator.h>

//Redundant
#include <ros/ros.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_floor_image_publisher");
        
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    egocylindrical::EgoCylinderFloorImageGenerator s(nh, pnh);
    s.init();
    //ros::MultiThreadedSpinner spinner(2);
    //spinner.spin();
    ros::spin();
}
