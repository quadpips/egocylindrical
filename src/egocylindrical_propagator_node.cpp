//
// Created by root on 2/5/18.
//

#include <egocylindrical/egocylindrical.h>

namespace egocylindrical
{


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_propagator");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    ROS_INFO_STREAM("Start");
    egocylindrical::EgoCylindricalPropagator s(nh, pnh);
    s.init();
    
    ros::spin();
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
}
