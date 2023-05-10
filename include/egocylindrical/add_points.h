#ifndef EGOCYLINDRICAL_ADD_POINTS_H
#define EGOCYLINDRICAL_ADD_POINTS_H

#include <egocylindrical/ecwrapper.h>

#include <ros/console.h>
#include <opencv2/core.hpp>

namespace egocylindrical
{

namespace utils
{

    
    inline
    void addPoints(utils::ECWrapper& cylindrical_history, const utils::ECWrapper& new_points, bool overwrite)
    {
        
        float* x = cylindrical_history.getX();
        float* y = cylindrical_history.getY();
        float* z = cylindrical_history.getZ();
        
        const float* n_x = new_points.getX();
        const float* n_y = new_points.getY();
        const float* n_z = new_points.getZ();
        
        const float* ranges = new_points.getRanges();
        const int32_t* inds = new_points.getInds();
        
        const bool use_egocan = (new_points.getParams().can_width > 0);

        ROS_DEBUG("Relocated the propagated image");
        //#pragma omp parallel for
        for(int i = 0; i < new_points.getNumPts(); ++i)
        {
            
            int idx=-1;
            idx= inds[i];
            
            if(idx >=0)
            {
                float depth = ranges[i];
                
                cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                
                float prev_depth = worldToRangeSquared(prev_point);
                
                if(!(prev_depth <= depth)) //overwrite || 
                {   
                    /*TODO: Check if this gets compiled out or not. If not, remove this object, 
                     * or perhaps use basic templated custom point class to combine benefits of
                     * readability and efficiency
                     */
                    cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
                    
                    x[idx] = world_pnt.x;
                    y[idx] = world_pnt.y;
                    z[idx] = world_pnt.z;
                    
                }
                
            }
            else if(use_egocan)
            {
                if(n_x[i]==n_x[i])  //Skip NaNs
                {
                    cv::Point3f world_pnt(n_x[i],n_y[i],n_z[i]);
                  
                    idx = new_points.worldToCanIdx(world_pnt);
                    
                    if(idx >=0 && idx < new_points.getNumPts())
                    {
                        float depth = worldToCanDepth(world_pnt);
                          
                        
                        cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                        
                        float prev_depth = worldToCanDepth(prev_point);
                        
                        if(!(prev_depth <= depth)) //overwrite || 
                        {   
                            
                            x[idx] = world_pnt.x;
                            y[idx] = world_pnt.y;
                            z[idx] = world_pnt.z;
                        }
                    }
                    else
                    {
                        ROS_WARN_STREAM("Invalid index [" << idx << "] for point (" << world_pnt.x << "," << world_pnt.y << "," << world_pnt.z << ")");
                    }
                  
                }
            }
            
            
            
        }
    
        
        
    }
    
    
}

}

#endif  //EGOCYLINDRICAL_ADD_POINTS_H
