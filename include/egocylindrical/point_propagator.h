#ifndef EGOCYLINDRICAL_POINT_PROPAGATOR_H
#define EGOCYLINDRICAL_POINT_PROPAGATOR_H

#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/point_transformer.h>
#include <tf2_ros/buffer.h>


namespace egocylindrical
{
/*    
    utils::ECParams getParams(const egocylindrical::PropagatorConfig &config)
    {
      utils::ECParams params;
      params.height = config.height;
      params.width = config.width;
      params.vfov = config.vfov;
      params.can_width = config.can_width;
      params.v_offset = config.v_offset;
      params.cyl_radius = config.cyl_radius;
      return params;
    }
    
    namespace utils
    {
      utils::ECWrapperPtr getECWrapper(const egocylindrical::PropagatorConfig &config, bool allocate_arrays=false)
      {
        return utils::getECWrapper(getParams(config), allocate_arrays);
      }
    }*/

namespace utils
{

    
    //Needs fixed_frame, buffer, transformed pts storage
    //To transform, just needs old points, desired frame_id (could actually move coordinate frame helper into this class), and new points storage
    class PointPropagator
    {
    public:
      PointPropagator(tf2_ros::Buffer& buffer):
        buffer_(buffer)
        {}
      
      bool init(std::string fixed_frame_id)
      {
        fixed_frame_id_ = fixed_frame_id;
        transformed_pts_ = utils::getECWrapper(utils::ECParams(), true);
        return true;
      }
      
      void transform(utils::ECWrapper& old_pnts, utils::ECWrapper& new_pnts, const std_msgs::Header& new_header, int num_threads)
      {
        ros::WallTime start = ros::WallTime::now();
        
        std_msgs::Header old_header = old_pnts.getHeader();
        
        new_pnts.setHeader(new_header);
        
        ROS_DEBUG("Getting Transformation details");
                geometry_msgs::TransformStamped trans = buffer_.lookupTransform(new_header.frame_id, new_header.stamp,
                                old_header.frame_id, old_header.stamp,
                                fixed_frame_id_);
        
        ROS_DEBUG_STREAM_NAMED("timing", "Finding transform took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
                
        
        start = ros::WallTime::now();    
        utils::transformPoints(old_pnts, *transformed_pts_, new_pnts, trans, num_threads);
        ROS_DEBUG_STREAM_NAMED("timing", "Transforming points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
        start = ros::WallTime::now();
        utils::addPoints(new_pnts, *transformed_pts_, false);
        ROS_DEBUG_STREAM_NAMED("timing", "Inserting transformed points took " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
      }
      
      
    protected:
      std::string fixed_frame_id_;
      tf2_ros::Buffer& buffer_;
      utils::ECWrapper::Ptr transformed_pts_;
    };
    
    
    //Only perform clearing if measurement frame_id has same origin as target frame
} //end namespace utils
    
} //end namespace egocylindrical

#endif //EGOCYLINDRICAL_POINT_PROPAGATOR_H
