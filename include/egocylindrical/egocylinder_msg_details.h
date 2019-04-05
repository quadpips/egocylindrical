#ifndef EGOCYLINDER_MSG_DETAILS_H
#define EGOCYLINDER_MSG_DETAILS_H

#include <egocylindrical/EgoCylinderPoints.h>
#include <boost/align/aligned_allocator.hpp>
#include <ros/message_forward.h>

namespace egocylindrical
{
    namespace utils
    {
        typedef boost::alignment::aligned_allocator<void, 32> AlignedAllocator;
        //ROS_DECLARE_MESSAGE_WITH_ALLOCATOR(EgoCylinderPoints, ECMsg, std::allocator<void> );
        
        //template <typename T>
        //using Allocator = std::allocator<void>;
        using Allocator = boost::alignment::aligned_allocator<void, __BIGGEST_ALIGNMENT__>;
        
        //typedef ::egocylindrical::EgoCylinderPoints_<Eigen::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        typedef ::egocylindrical::EgoCylinderPoints_<utils::Allocator> ECMsg;
        
        // NOTE: I'm not sure that using this typedef renamed version was such a good idea after all...
        //typedef AlignedEgoCylinderPoints ECMsg;
        //typedef EgoCylinderPoints ECMsg;
        typedef boost::shared_ptr<ECMsg> ECMsgPtr;
        typedef boost::shared_ptr<ECMsg const> ECMsgConstPtr;
    }
    


}

#endif //EGOCYLINDER_MSG_DETAILS_H
