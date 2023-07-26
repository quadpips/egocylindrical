#ifndef EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
#define EGOCYLINDRICAL_ECWRAPPER_BUFFER_H

#include <egocylindrical/PropagatorConfig.h>
#include <egocylindrical/ecwrapper.h>

#include <stdexcept>
#include <queue>

namespace egocylindrical
{

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
    }


    class ECWrapperBuffer
    {
    public:
        ECWrapperBuffer(egocylindrical::PropagatorConfig& config):
            config_(config)
        {}

        bool init()
        {
            old_pts_buffer_.push(nullptr);
            addNew();
            return true;
        }

        utils::ECWrapper::Ptr getOld()
        {
            if(!old_pts_buffer_.empty())
            {
                return old_pts_buffer_.front();
            }
            else
            {
                ROS_ERROR_STREAM("There should at least be a nullptr in [old_pts_buffer_]");
                throw std::out_of_range("There should at least be a nullptr in [old_pts_buffer_]");
            }
            // if(!old_pts_buffer_.empty())
            // {
            //     return old_pts_buffer_.front();
            // }
            // else
            // {
            //     return nullptr;
            // }
            // try
            // {
            //     return old_pts_buffer_.front();
            // } catch (std::out_of_range& e)
            // {
            //     ROS_ERROR_STREAM("");
            // }
        }

        void addNew()
        {
             next_pts_buffer_.push(nullptr);
             prepareNext();
        }

        // void addIfNeeded()
        // {
        //     if(buffer_.empty())
        //     {
        //         addNew();
        //     }
        // }

        utils::ECWrapper::Ptr getNew()
        {
            if(!new_pts_buffer_.empty())
            {
                return new_pts_buffer_.front();
            }
            else
            {
                ROS_ERROR_STREAM("There must be a valid ECWrapper in [new_pts_buffer_]");

                throw std::out_of_range("There must be a valid ECWrapper in [new_pts_buffer_]");
            }
//             if(!new_pts_buffer_.empty())
//             {
//
//             }
//             else
//             {
//                 ROS_WARN_STREAM("New ECWrapper should already have been constructed!");
//                 return nullptr;
//             }
        }

        void update()
        {
            releaseOld();
            makeNewOld();
            prepareNext();
        }


    protected:
        void releaseOld()
        {
            // try
            // {
                auto v = getOld();
                old_pts_buffer_.pop();
                next_pts_buffer_.push(v);
                if(!v)
                {
                    ROS_DEBUG_STREAM("Adding nullptr ECWrapper");
                }
                else if(!v->isLocked())
                {
                    ROS_DEBUG_STREAM("Adding old unlocked ECWrapper");
                }
                else
                {
                    ROS_DEBUG_STREAM("Adding old locked ECWrapper");
                }
            // }
            // catch (std::out_of_range& e)
            // {
            //     ROS_ERROR_STREAM("There should at least be a nullptr in [old_pts_buffer_]");
            //     throw;
            // }
        }

        void makeNewOld()
        {
            auto v = getNew();
            new_pts_buffer_.pop();
            old_pts_buffer_.push(v);
        }

        // void makeOldNew()
        // {
        //     auto v = getOld();
        //
        //     if(v->isLocked())
        //     {
        //
        //         new_pts_buffer_.
        //
        //     }
        // }

        utils::ECWrapper::Ptr createNew()
        {
            return utils::getECWrapper(config_);
        }

        //In the future, this can be done by separate thread as triggered by condition variable
        void prepareNext()
        {
            while(!next_pts_buffer_.empty())
            {
                auto v = next_pts_buffer_.front();
                next_pts_buffer_.pop();

                if(v)
                {
                    if(!v->isLocked())
                    {
                        v->init(getParams(config_), true);
                        ROS_DEBUG_STREAM("Reuse old ECWrapper for next time");
                    }
                    else
                    {
                        v = createNew();
                        ROS_DEBUG_STREAM("Cannot reuse old locked ECWrapper, create new one");
                    }
                }
                else
                {
                    v = createNew();
                    ROS_DEBUG_STREAM("No old ECWrapper to reuse, create new one");
                }

                new_pts_buffer_.push(v);
            }

        }



    protected:
        egocylindrical::PropagatorConfig& config_;

        std::queue<utils::ECWrapper::Ptr> new_pts_buffer_, old_pts_buffer_, next_pts_buffer_;
    };

    // class ECWrapperUpdateLogic
    // {
    //
    // public:
    //     ECWrapperUpdateLogic(egocylindrical::PropagatorConfig& config):
    //         config_(config)
    //         buffers_(config)
    //     {}
    //
    //     void update(std_msgs::Header target_header, utils::SensorMeasurement& measurement)
    //     {
    //         utils::ECWrapper::Ptr old_pts = buffers_.getOld();
    //         utils::ECWrapper::Ptr new_pts;
    //         if(old_pts && measurement.raytrace)
    //         {
    //             new_pts = buffers_.getNew();
    //             pp_.transform(*old_pts, *new_pts, target_header, config_.num_threads);
    //         }
    //     }
    //
    // protected:
    //     egocylindrical::PropagatorConfig& config_;
    //     ECWrapperBuffer buffers_;
    // };

}   //end namespace egocylindrical

#endif //EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
