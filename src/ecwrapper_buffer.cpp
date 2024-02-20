#include <egocylindrical/ecwrapper_buffer.h>
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



    ECWrapperBuffer::ECWrapperBuffer(egocylindrical::PropagatorConfig& config):
        config_(config),
        old_pts_(nullptr)
    {}

    bool ECWrapperBuffer::init()
    {
        ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.init", "Initialize ECWrapperBuffer");
        // old_pts_buffer_.push(nullptr);
        addNew();
        return true;
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::getOld()
    {
        return old_pts_;
    }

    void ECWrapperBuffer::addNew()
    {
            next_pts_buffer_.push(nullptr);
            prepareNext();
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::getNew()
    {
        if(!new_pts_buffer_.empty())
        {
            ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.getNew", "Retrieving clean, preallocated ECWrapper");
            new_pts_ = new_pts_buffer_.front();
            new_pts_buffer_.pop();
            return new_pts_;
        }
        else
        {
            ROS_ERROR_STREAM("There must be a valid ECWrapper in [new_pts_buffer_]");

            throw std::out_of_range("There must be a valid ECWrapper in [new_pts_buffer_]");
        }
    }

    //utils::ECWrapper::Ptr updatePoints(utils::ECWrapper::Ptr old_pts)



    utils::ECWrapper::Ptr ECWrapperBuffer::reuseOld()
    {
        auto old = old_pts_;
        if(old->isLocked())
        {
            new_pts_ = copyECWrapper(*old);
            // old_pts_buffer_.pop();
            //releaseOld();
            ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.reuseOld", "Unable to reuse old locked ECWrapper, copying it");
        }
        else
        {
            new_pts_ = old_pts_;
            old_pts_ = nullptr;
            ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.reuseOld", "Reusing old unlocked ECWrapper");
        }
        return new_pts_;
    }

    void ECWrapperBuffer::update()
    {
        //releaseOld();
        makeNewOld();
        prepareNext();

        ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.update", "new_pts_buffer: " << new_pts_buffer_.size() << "; next_pts_buffer: " << next_pts_buffer_.size());
    }


    void ECWrapperBuffer::releaseOld()
    {
        auto v = old_pts_;
        // if(v)
        {
            next_pts_buffer_.push(v);
            if(!v)
            {
                ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing nullptr ECWrapper");
            }
            else if(!v->isLocked())
            {
                ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing old unlocked ECWrapper");
            }
            else
            {
                ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing old locked ECWrapper");
            }
        }
        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "No old ECWrapper");
    }


    void ECWrapperBuffer::makeNewOld()
    {
        old_pts_ = new_pts_;
        // new_pts_buffer_.pop();
        new_pts_ = nullptr;
        ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.makeNewOld", "'New' becomes 'old', 'New' becomes nullptr");
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::createNew()
    {
        ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.createNew", "Create new ECWrapper with current config");
        return utils::getECWrapper(config_);
    }

    //In the future, this can be done by separate thread as triggered by condition variable
    void ECWrapperBuffer::prepareNext()
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
                    ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.prepareNext", "Reuse old ECWrapper for next time");
                }
                else
                {
                    v = createNew();
                    ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.prepareNext", "Cannot reuse old locked ECWrapper, create new one");
                }
            }
            else
            {
                v = createNew();
                ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.makprepareNexteNewOld", "No old ECWrapper to reuse, create new one");
            }

            new_pts_buffer_.push(v);
        }

    }

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
