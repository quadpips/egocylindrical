#ifndef EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
#define EGOCYLINDRICAL_ECWRAPPER_BUFFER_H

#include <egocylindrical/PropagatorConfig.h>
#include <egocylindrical/ecwrapper.h>

#include <queue>

namespace egocylindrical
{

    class ECWrapperBuffer
    {


    public:
        ECWrapperBuffer(egocylindrical::PropagatorConfig& config);

        bool init();

        utils::ECWrapper::Ptr getOld();

        void addNew();

        utils::ECWrapper::Ptr getNew();

        utils::ECWrapper::Ptr reuseOld();

        void update();

    public:

        void releaseOld();

    protected:

        void makeNewOld();

        utils::ECWrapper::Ptr createNew();

        //In the future, this can be done by separate thread as triggered by condition variable
        void prepareNext();

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
