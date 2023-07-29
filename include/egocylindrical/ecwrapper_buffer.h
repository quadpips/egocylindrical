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
            config_(config),
            old_pts_(nullptr)
        {}

        bool init()
        {
            // old_pts_buffer_.push(nullptr);
            addNew();
            return true;
        }

        utils::ECWrapper::Ptr getOld()
        {
            return old_pts_;
        }

        void addNew()
        {
             next_pts_buffer_.push(nullptr);
             prepareNext();
        }

        utils::ECWrapper::Ptr getNew()
        {
            if(!new_pts_buffer_.empty())
            {
                new_pts_ = new_pts_buffer_.front();
                return new_pts_;
            }
            else
            {
                ROS_ERROR_STREAM("There must be a valid ECWrapper in [new_pts_buffer_]");

                throw std::out_of_range("There must be a valid ECWrapper in [new_pts_buffer_]");
            }
        }

        //utils::ECWrapper::Ptr updatePoints(utils::ECWrapper::Ptr old_pts)



        utils::ECWrapper::Ptr reuseOld()
        {
            auto old = old_pts_;
            if(old->isLocked())
            {
                new_pts_ = copyECWrapper(*old);
                // old_pts_buffer_.pop();
                //releaseOld();
            }
            else
            {
                new_pts_ = old_pts_;
                old_pts_ = nullptr;
            }
            return new_pts_;
        }

        void update()
        {
            //releaseOld();
            makeNewOld();
            prepareNext();
        }


    public:

        void releaseOld()
        {
            auto v = old_pts_;
            if(v)
            {
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
            }
        }

    protected:

        void makeNewOld()
        {
            old_pts_ = new_pts_;
            // new_pts_buffer_.pop();
            new_pts_ = nullptr;
        }

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

    public:
        using Ptr = std::shared_ptr<ECWrapperBuffer>;


    protected:
        egocylindrical::PropagatorConfig& config_;

        std::queue<utils::ECWrapper::Ptr> new_pts_buffer_, next_pts_buffer_; //old_pts_buffer_,

        utils::ECWrapper::Ptr old_pts_, new_pts_;
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
