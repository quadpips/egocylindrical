#ifndef NP_EGOCYLINDER_POINT_TRANSFORMER_H
#define NP_EGOCYLINDER_POINT_TRANSFORMER_H


#include <np_egocylinder/ecwrapper.h>
#include <geometry_msgs/TransformStamped.h>


namespace np_egocylinder
{
    
    namespace utils
    {

      //void transformPoints(utils::ECWrapper& points, const geometry_msgs::TransformStamped& trans);
      
      void transformPoints(const utils::ECWrapper& points, utils::ECWrapper& transformed_points, const utils::ECWrapper& new_points, const geometry_msgs::TransformStamped& trans, int num_threads=1);

    }

}

#endif //EGOCYLINDRICAL_POINT_TRANSFORMER_H
