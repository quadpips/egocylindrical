#ifndef EGOCYLINDRICAL_ECSTIXEL_H
#define EGOCYLINDRICAL_ECSTIXEL_H



#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <stixel_estimator/stixelListMsg.h>
#include <stixel_estimator/stixelMsg.h>

#include <image_geometry/pinhole_camera_model.h>



namespace egocylindrical
{
    
    namespace utils
    {

        constexpr float dNaNs=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        float inf = std::numeric_limits<float>::infinity();
        
        
        /*
         * This class is intended to act as an abstraction of the egocylindrical representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        
        class ECStixel
        {
        private:
            std::vector<stixel_estimator::stixelMsg> stixels_;
            std::vector<float> x;
            std::vector<float> bottom_y;
            std::vector<float> top_y;
            std::vector<float> z;
            
            int height_, width_;
//             float vfov_;
            
            std_msgs::Header header_;
            stixel_estimator::stixelListMsg::Ptr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            stixel_estimator::stixelListMsg::ConstPtr const_msg_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            ECStixel(int height, int width):
            height_(height),
            width_(width)
            {
                msg_ = boost::make_shared<stixel_estimator::stixelListMsg>();
                msg_->stixels.resize(width_);  //Note: can pass 'utils::dNaN as 2nd argument to set all values
                stixels_ = msg_->stixels;
                x.resize(width_, utils::dNaNs);
                bottom_y.resize(width_, utils::dNaNs);
                top_y.resize(width_, utils::dNaNs);
                z.resize(width_, utils::dNaNs);
            }
            
//             ECStixel(const stixel_estimator::stixelListMsgConstPtr& ec_stixels) 
//             {
//                 const_msg_ = ec_stixels;
//                 header_ = const_msg_->header;
//                 x = 
//                 
//                 //std::cout << "Address: " << std::hex  << const_msg_->points.data.data() << std::dec << ", height=" << height_ << ", width=" << width_ << ", step=" << step << std::endl;
//                 
//             }
            
            inline
            bool generateStixel(const image_geometry::PinholeCameraModel& cam_model, const float baseline)
            {
                for(int i = 0; i < stixels_.size(); ++i)
                {
                    cv::Point3f world_pt_b(x[i], bottom_y[i], z[i]);
                    cv::Point img_pt_b = cam_model.project3dToPixel(world_pt_b);
                    cv::Point3f world_pt_t(x[i], top_y[i], z[i]);
                    cv::Point img_pt_t = cam_model.project3dToPixel(world_pt_t);
                    
                    stixels_[i].x = img_pt_b.x;
                    stixels_[i].bottom_y = img_pt_b.y;
                    stixels_[i].top_y = img_pt_t.y;
                    stixels_[i].depth = z[i];
                    stixels_[i].isValid = 1;
                    if(stixels_[i].depth == utils::dNaNs)
                    {
                        stixels_[i].depth = inf;
                        stixels_[i].disparity = 0;
                    }
                    else if(stixels_[i].depth == 0)
                    {
                        stixels_[i].disparity = inf;
                    }
                    else
                    {
                        stixels_[i].disparity = baseline * cam_model.fx() / stixels_[i].depth;
                    }
                    
                }
                return true;
            }
                    
            
            
            
            inline std::vector<stixel_estimator::stixelMsg>* getStixels()                   { return &stixels_; }
            inline const std::vector<stixel_estimator::stixelMsg>* getStixels()     const   { return &stixels_; }
            
            inline std::vector<float>* getX()                   { return &x; }
            inline const std::vector<float>* getX()     const   { return &x; }
            inline std::vector<float>* getBY()                  { return &bottom_y; }
            inline const std::vector<float>* getBY()    const   { return &bottom_y; }
            inline std::vector<float>* getTY()                  { return &top_y; }
            inline const std::vector<float>* getTY()    const   { return &top_y; }
            inline std::vector<float>* getZ()                   { return &z; }
            inline const std::vector<float>* getZ()     const   { return &z; }

            inline
            void setHeader(std_msgs::Header header)
            {
                header_ = header;
                msg_->header = header;
            }
            
            inline
            int getSize() const
            {
                return stixels_.size();
            }
            
            inline
            int getWidth() const
            {
                return width_;
            }
            
            inline
            int getHeight() const
            {
                return height_;
            }
            
            inline
            std_msgs::Header getHeader() const
            {
                return header_; 
            }
            
            inline
            int getStixelRange() const
            {
                return width_;
            }
            
            inline
            void generateStixelMsg()
            {
                msg_->stixels = stixels_;
                msg_->header = header_;
            }
            
            inline
            stixel_estimator::stixelListMsg::ConstPtr getECStixelMsg()
            {
                /* This is a temporary solution until I can decide how to do this. Ideally, I would publish the final message directly
                 * so that other nodelets can do their thing. However, currently I transform points in place, overwriting the old values.
                 * The whole point of storing the data in a msgptr may be undermined by this. It will come down to whether it is faster to
                 * 1. Copy the message here before publishing
                 * 2. Publish the message, then make a copy for use in callback
                 * 3. Perform out of place point transformation
                 */
                stixel_estimator::stixelListMsg::ConstPtr msg = boost::make_shared<stixel_estimator::stixelListMsg>(*msg_);
                
                return (stixel_estimator::stixelListMsg::ConstPtr) msg_;
            }
            
            // Function stubs to fill in and uncomment as needed
            /*
             *    inline
             *    float& x(int row, int col)
             *    {
             *        
        }
        
        inline
        float& y(int row, int col)
        {
        
        }
        
        inline
        float& z(int row, int col)
        {
        
        }
        
        inline
        const float& x(int row, int col) const
        {
        return (const float& ) x(row, col);
        }
        
        inline
        const float& y(int row, int col) const
        {
        return (const float& ) y(row, col);
        }
        
        inline
        const float& z(int row, int col) const
        {
        return (const float& ) z(row, col);
        }
        
        inline
        cv::Point3f at(int row, int col)
        {
        
        }
        
        */
            
            
        };
        
        typedef std::shared_ptr<ECStixel> ECStixelPtr;
        
        inline
        ECStixelPtr getECStixel(int height, int width)
        {
            return std::make_shared<ECStixel>(height,width);
        }
        
    }
}
    
#endif //EGOCYLINDRICAL_ECSTIXEL_H
