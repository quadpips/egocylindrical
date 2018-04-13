#ifndef EGOCYLINDRICAL_ECWRAPPER_H
#define EGOCYLINDRICAL_ECWRAPPER_H



#include <ros/ros.h>
#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf/LinearMath/Matrix3x3.h>
//#include <omp.h>
//#include <sensor_msgs/PointCloud2.h>

#include <egocylindrical/EgoCylinderPoints.h>

//#include <eigen_stl_containers/eigen_stl_containers.h>

#include <boost/align/aligned_alloc.hpp>
#include <boost/align/aligned_allocator.hpp>

#include <cstddef>
#include <cstdalign>

//#include <iomanip> // for debug printing


namespace egocylindrical
{
    
    namespace utils
    {

        constexpr float dNaN=(std::numeric_limits<float>::has_quiet_NaN) ? std::numeric_limits<float>::quiet_NaN() : 0;
        
        // TODO: Quantify max error and verify that it doesn't affect anything
        // Source: https://gist.github.com/volkansalma/2972237
        inline
        float atan2_approximation1(float y, float x)
        {
            //http://pubs.opengroup.org/onlinepubs/009695399/functions/atan2.html
            //Volkan SALMA
            
            constexpr float ONEQTR_PI = M_PI / 4.0;
            constexpr float THRQTR_PI = 3.0 * M_PI / 4.0;
            float r, angle;
            float abs_y = fabs(y) + 1e-10f;      // kludge to prevent 0/0 condition
            if ( x < 0.0f )
            {
                r = (x + abs_y) / (abs_y - x);
                angle = THRQTR_PI;
            }
            else
            {
                r = (x - abs_y) / (x + abs_y);
                angle = ONEQTR_PI;
            }
            angle += (0.1963f * r * r - 0.9817f) * r;
            if ( y < 0.0f )
                return( -angle );     // negate if in quad III or IV
                else
                    return( angle );
                
                
        }
        
        template <typename T>
        inline
        cv::Point3_<T> projectWorldToCylinder(const cv::Point3_<T>& point)
        {
          cv::Point3_<T> Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
          return Pcyl_t;
        }
        
        inline
        cv::Point3f projectWorldToCylinder(const cv::Point3f& point)
        {
            cv::Point3f Pcyl_t = point / std::sqrt(point.x * point.x + point.z * point.z);
            return Pcyl_t;
        }
        
        
        inline
        cv::Point2f worldToCylindricalImage(const cv::Point3f& point, int cyl_width, int cyl_height, float h_scale, float v_scale, float h_offset, float v_offset)
        {
            
            cv::Point3f p_cyl = projectWorldToCylinder(point);
            
            float x = atan2_approximation1(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
            //float x = std::atan2(p_cyl.x, p_cyl.z) * h_scale + cyl_width / 2;
            float y = p_cyl.y * v_scale + cyl_height / 2;
            
            cv::Point2f im_pt(x,y);
            return im_pt;
        }
        
        template <typename T>
        inline
        T worldToRangeSquared(const T x, const T z)
        {
            return x*x + z*z;
        }
        
        template <typename T>
        inline
        T worldToRangeSquared(const cv::Point3_<T>& point)
        {
            return point.x*point.x + point.z*point.z;
        }
        
        template <typename T>
        inline
        T worldToRange(const cv::Point3_<T>& point)
        {
            return std::sqrt(worldToRangeSquared(point));
        }
        
        

        
        
        //typedef ::egocylindrical::EgoCylinderPoints_<Eigen::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        typedef ::egocylindrical::EgoCylinderPoints_<boost::alignment::aligned_allocator<void, 32> > AlignedEgoCylinderPoints;
        
        // NOTE: I'm not sure that using this typedef renamed version was such a good idea after all...
        //typedef AlignedEgoCylinderPoints ECMsg;
        typedef EgoCylinderPoints ECMsg;
        typedef boost::shared_ptr<ECMsg> ECMsgPtr;
        typedef boost::shared_ptr<ECMsg const> ECMsgConstPtr;
        
        template <typename T>
        using AlignedVector = std::vector<T, boost::alignment::aligned_allocator<T, __BIGGEST_ALIGNMENT__> >;
        
        
        
        // This class serves as the converter between egocylindrical and world coordinates
        class ECConverter
        {
        protected:
          int height_, width_;
          float vfov_;
          float hscale_, vscale_;
          
        public:
          ECConverter()
          {
          }
          
          inline
          void updateParams(int height, int width, float vfov)
          {
            height_ = height;
            width_ = width;
            vfov_ = vfov;
            hscale_ = width_/(2*M_PI);
            vscale_ = height_/vfov_;
          }
          
          ECConverter(int height, int width, float vfov)
          {
            updateParams(height,width,vfov);
          }
          
          // NOTE: Not sure whether this belongs here or in derived class
          inline
          void fromCameraInfo(const ECMsgConstPtr& msg)
          {
            
            float vfov = msg->fov_v;
            
            const std::vector<std_msgs::MultiArrayDimension>& dims = msg->points.layout.dim;
            int components = dims[0].size;
            int height = dims[1].size;
            int width = dims[2].size;
            
            updateParams(height, width, vfov);
          }
          
          ECConverter(const ECMsgConstPtr& msg)
          {
            fromCameraInfo(msg);
          }
          
          inline
          cv::Point worldToCylindricalImage(const cv::Point3f& point) const
          {
            return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
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
          
          template <typename T>
          inline 
          cv::Point_<T> project3dToPixel(const cv::Point3_<T> point)
          {
            return utils::worldToCylindricalImage(point, width_, height_, hscale_, vscale_, 0, 0);
          }
          
          
          inline
          cv::Point3d projectPixelTo3dRay(const cv::Point2d& point) const
          {
            cv::Point3d ray;
            double theta = (point.x - (width_/2))/hscale_;
            
            ray.x = sin(theta);
            ray.z = cos(theta);
            
            ray.y = (point.y - (height_/2))/vscale_;
            
            ray /= (ray.x*ray.x + ray.y * ray.y + ray.z*ray.z);
            
            return ray;
          }
          
        };
        
        /*
         * This class is intended to act as an abstraction of the egocylindrical data representation
         * to enable other functions to operate on it without requiring knowledge of the implementation.
         * Functionality will be moved incrementally.
         * It is hoped that it will be extended to simplify other egocylindrical versions, ex. stixel
         */
        class ECPointInterface
        {
        public:
          inline virtual float* getX()                        =0;
          inline virtual const float* getX()          const   =0;
          
          inline virtual float* getY()                        =0;
          inline virtual const float* getY()          const   =0;
          
          inline virtual float* getZ()                        =0;
          inline virtual const float* getZ()          const   =0;

          inline virtual int getNumPts()              const   =0;
          inline virtual int getCols()                const {return getNumPts(); }

        };
        
        class ECPropagationTemporaries
        {
        private:
          
          AlignedVector<float> ranges_;       
          AlignedVector<long int> inds_; // Note: on 32/64 bit systems, the 'long' is generally redundant as 'int' almost always has the same size as 'long', but just to be safe...
          
        public:
          inline
          void resize(int size)
          {
            ranges_.resize(size);
            inds_.resize(size);
          }
          
          ECPropagationTemporaries() {}
          
          ECPropagationTemporaries(int size)
          {
            resize(size);
          }
          
          inline float* getRanges()                 __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (float*)           ranges_.data(); }
          inline const float* getRanges()  const    __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (const float*)     ranges_.data(); }
          
          inline long int* getInds()                __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return                    inds_.data(); }
          inline const long int* getInds() const    __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (const long int*)  inds_.data(); }
          
        };
        
        
        class ECAlignedPointsStorage : public ECPointInterface
        {
        private:
          
          AlignedVector<float> x_;       
          AlignedVector<float> y_;
          AlignedVector<float> z_;
          
        public:
          inline
          void resize(int size)
          {
            x_.resize(size, dNaN);
            y_.resize(size, dNaN);
            z_.resize(size, dNaN);
          }
          
          ECAlignedPointsStorage() {}
          
          ECAlignedPointsStorage(int size)
          {
            resize(size);
          }
          
          inline float* getX()                 __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (float*)           x_.data(); }
          inline const float* getX()  const    __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (const float*)     x_.data(); }
          
          inline float* getY()                 __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (float*)           y_.data(); }
          inline const float* getY()  const    __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (const float*)     y_.data(); }
          
          inline float* getZ()                 __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (float*)           z_.data(); }
          inline const float* getZ()  const    __attribute__((assume_aligned(__BIGGEST_ALIGNMENT__)))      { return (const float*)     z_.data(); }
          
          inline int getNumPts()      const     { return x_.size(); };
          
        };
        
        
        class ECPropagationResult : public ECPropagationTemporaries, public ECAlignedPointsStorage
        {
        public:
          ECPropagationResult() {}
          
          inline
          void resize(int size)
          {
            ECPropagationTemporaries::resize(size);
            ECAlignedPointsStorage::resize(size);
          }
        };
        
        
        /*
         * ECWrapper is the interface for accessing egocylindrical data stored in a EgoCylinderPoints message
         */
        
        class ECWrapper : public ECConverter, public ECPointInterface, public ECPropagationTemporaries
        {
        private:

            float* points_;

            std_msgs::Header header_;
            ECMsgPtr msg_; // The idea would be to store everything in the message's allocated storage to prevent copies
            
            ECMsgConstPtr const_msg_;
            
            // For now, just get things working using this.
            // Note: It may be preferable to allocate x,y,z separately to ensure they are aligned (unless the width is chosen such that they will be anyway...)
            // Another idea: possibly template this class by height/width, potentially enabling compile time optimizations
            // Also: maybe should store x, then z, then y, since only x and z are needed for range image
           

           
        public:
            
            ECWrapper(int height, int width, float vfov, bool allocate_arrays = false):
              ECConverter(height, width, vfov)
            {
                msg_ = boost::make_shared<ECMsg>();
                
                //Eigen::aligned_allocator<EgoCylinderPoints> Alloc;
                //msg_ = boost::allocate_shared<EgoCylinderPoints>(Alloc);
                
                int max_alignment = alignof(std::max_align_t);
                
                int biggest_alignment = __BIGGEST_ALIGNMENT__;
                
                size_t object_size = sizeof(float);
                
                size_t object_alignment = alignof(float);
                
                size_t buffer_size = biggest_alignment - object_size;
                size_t buffer_objects = buffer_size / object_size;
                
                // NOTE: width x height must be divisible by 8 for this approach to work. Otherwise, additional information will be necessary in order to properly place the x,y,z pointers
                
                ROS_INFO_STREAM("max_alignment: " << max_alignment << ", biggest_alignment: " << biggest_alignment << ", object_size: " << object_size << ", object_alignment: " << object_alignment << ", number buffer objects: " << buffer_objects);

                msg_->points.data.resize(3*height_*width_ + buffer_objects, dNaN);  //Note: can pass 'utils::dNaN as 2nd argument to set all values
                
                
                points_ = msg_->points.data.data();
                
                // TODO: create templated function to get aligned pointers. Something similar here: http://en.cppreference.com/w/cpp/memory/align
                
                {
                    void* temp_points = (void*) msg_->points.data.data();
                    
                    size_t space_before = height_*width_*3*sizeof(float);
                    size_t space_after = space_before;
                    
                    std::align(biggest_alignment, sizeof(float), temp_points, space_after);
                    points_ = (float*) temp_points;
                    
                    msg_->points.layout.data_offset = (space_before - space_after);
                    
                    
                    ROS_INFO_STREAM("Aligned points_, adjusted pointer by " << (space_before - space_after) << " bytes");
                }
                
                
                
                if(allocate_arrays)
                {
                  ECPropagationTemporaries::resize(height_*width_);
                }
                
                msg_->fov_v = vfov_;
                
                
                std::vector<std_msgs::MultiArrayDimension>& dims = msg_->points.layout.dim;
                dims.resize(3);
                
                
                std_msgs::MultiArrayDimension& dim0 = dims[0];
                dim0.label = "components";
                dim0.size = 3;
                dim0.stride = 3*height_*width_;
                
                
                std_msgs::MultiArrayDimension& dim1 = dims[1];
                dim1.label = "rows";
                dim1.size = height_;
                dim1.stride = height_*width_;

                
                std_msgs::MultiArrayDimension& dim2 = dims[2];
                dim2.label = "point";
                dim2.size = width_;
                dim2.stride = width_;             

            }
            
            ECWrapper(const ECMsgConstPtr& ec_points) :
              ECConverter(ec_points)
            {
                const_msg_ = ec_points;
                header_ = const_msg_->header;
                                
                points_ = (float*) const_msg_->points.data.data() + (const_msg_->points.layout.data_offset) / sizeof(float);
                
                //std::cout << "Address: " << std::hex  << const_msg_->points.data.data() << std::dec << ", height=" << height_ << ", width=" << width_ << ", step=" << step << std::endl;
                
            }
            
            ~ECWrapper()
            {

            }
            
                        
            inline float* getPoints()                   { return (float*) points_; }
            inline const float* getPoints()     const   { return (const float*) points_; }
            
            inline float* getX()                        { return getPoints(); }
            inline const float* getX()          const   { return (const float*) getPoints(); }
            
            inline float* getY()                        { return getPoints() + (height_ * width_); }
            inline const float* getY()          const   { return (const float*) getPoints() + (height_ * width_); }
            
            inline float* getZ()                        { return getPoints() + 2*(height_ * width_); }
            inline const float* getZ()          const   { return (const float*) getPoints() + 2*(height_ * width_); }

            inline int getNumPts()      const   { return height_ * width_; }
            

            inline
            void setHeader(std_msgs::Header header)
            {
                header_ = header;
                msg_->header = header;
            }
            

            inline
            std_msgs::Header getHeader() const
            {
                return header_; 
            }
            
            inline
            cv::Rect getImageRoi() const
            {
                return cv::Rect(0, 0, width_, height_);
            }
            
            
            inline
            int worldToCylindricalIdx(float x, float y, float z) const
            {
                cv::Point image_pnt = worldToCylindricalImage(cv::Point3f(x,y,z));
                
                int tidx = image_pnt.y * getWidth() +image_pnt.x;
                
                return tidx;
            }
            
            
            inline
            ECMsgConstPtr getEgoCylinderPointsMsg()
            {
                /* This is a temporary solution until I can decide how to do this. Ideally, I would publish the final message directly
                 * so that other nodelets can do their thing. However, currently I transform points in place, overwriting the old values.
                 * The whole point of storing the data in a msgptr may be undermined by this. It will come down to whether it is faster to
                 * 1. Copy the message here before publishing
                 * 2. Publish the message, then make a copy for use in callback
                 * 3. Perform out of place point transformation
                 */
                //ECMsgConstPtr msg = boost::make_shared<EgoCylinderPoints>(*msg_);
                
                return (ECMsgConstPtr) msg_;
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
        
        typedef std::shared_ptr<ECWrapper> ECWrapperPtr;
        
        inline
        ECWrapperPtr getECWrapper(int height, int width, float vfov, bool allocate_arrays=false)
        {
            return std::make_shared<ECWrapper>(height,width,vfov,allocate_arrays);
        }
        
    }
}
    
#endif //EGOCYLINDRICAL_ECWRAPPER_H