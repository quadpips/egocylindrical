#ifndef EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
#define EGOCYLINDRICAL_ECWRAPPER_BUFFER_H

#include <egocylindrical/PropagatorConfig.h>
#include <egocylindrical/ecwrapper.h>

// #include <egocylindrical/ts_queue.h>

#include <queue>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>

// #include <boost/thread/shared_mutex.hpp>
// #include <boost/thread/locks.hpp>



namespace egocylindrical
{

    class ECWrapperBuffer
    {

    private:

        // typedef boost::shared_mutex Mutex;
        using Mutex = std::mutex;
        // typedef boost::unique_lock< Mutex > WriteLock;
        // typedef boost::shared_lock< Mutex > ReadLock;
        using Lock = std::unique_lock<Mutex>;
        Mutex config_mutex_, reset_mutex_;

        using ConditionVar = std::condition_variable;
        Mutex next_pts_mutex_;
        ConditionVar next_pts_cv_;

        Mutex old_pnts_mutex_;
        ConditionVar old_pnts_cv_;
        bool reset_requested_;

        using Thread = std::thread;
        using ThreadPtr = std::unique_ptr<Thread>;
        ThreadPtr processing_thread_;


    public:
        ECWrapperBuffer(egocylindrical::PropagatorConfig& config);

        ~ECWrapperBuffer();

        bool init();

        void reset(float block_time=0);

        utils::ECWrapper::Ptr getOld();

        void addNew();

        void addToBuffer(utils::ECWrapper::Ptr v);

        utils::ECWrapper::Ptr getNew();

        utils::ECWrapper::Ptr reuseOld();

        void update();

    public:

        void releaseOld();

    protected:

        void makeNewOld();

        utils::ECWrapper::Ptr createNew();

        //In the future, this can be done by separate thread as triggered by condition variable
        // void prepareNext();

        void bufferProcessingThread();

    public:
        using Ptr = std::shared_ptr<ECWrapperBuffer>;


    protected:
        egocylindrical::PropagatorConfig& config_;

        std::queue<utils::ECWrapper::Ptr> new_pts_buffer_, next_pts_buffer_; //old_pts_buffer_,
        // TSQueue<utils::ECWrapper::Ptr> new_pts_buffer_, next_pts_buffer_;
        
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
