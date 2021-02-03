//
// Created by root on 2/5/18.
//

#include <np_egocylinder/egocylindrical.h>

namespace np_egocylinder
{


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "egocylindrical_propagator");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    np_egocylinder::EgoCylindricalPropagator s(nh, pnh);
    s.init();
    
    ros::spin();
    ros::MultiThreadedSpinner spinner;
    spinner.spin();
}
