#include <egocylindrical/laser_scan_inserter.h>
#include <egocylindrical/point_transformer_object.h>
#include <numeric>

namespace egocylindrical
{
    namespace utils
    {
      
        class LaserScan2Points
        {
        public:
          
            LaserScan2Points(){}
            
            virtual void setScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)=0;
            
            virtual cv::Point3f getPoint(int ind) const = 0;
            
        };
        
        
        void getUnitVector(float angle_min, float angle_max, float angle_increment, unsigned int length, int index, float& vx, float& vy)  //NOTE: 'angle_max' is not used
        {
          vx = std::cos(angle_min + (float) index * angle_increment);
          vy = std::sin(angle_min + (float) index * angle_increment);
        };
        
        
        void getUnitVector(const sensor_msgs::LaserScan& scan_msg, int index, float& vx, float& vy)
        {
            getUnitVector(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment, scan_msg.ranges.size(), index, vx, vy);
        }
        
        
        class BasicLaserScan2Points: public LaserScan2Points
        {
        public:
          
            virtual void setScan(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
            {
                scan_msg_ = scan_msg;
            }
          
            
            virtual cv::Point3f getPoint(int ind) const
            {
                const auto& ranges = scan_msg_->ranges;
                
                float vx, vy;
                getUnitVector(*scan_msg_, ind, vx, vy);
                float x = ranges[ind] * vx;
                float y = ranges[ind] * vy;
                return cv::Point3f(x,y,0);
            }
          
        protected:            
            sensor_msgs::LaserScan::ConstPtr scan_msg_;
          
        };
        
        
        
        int getInd(const ECWrapper& cylindrical_points, const cv::Point3f& pnt)
        { 
            int idx = -1;
            const int max_ind = cylindrical_points.getCols();

            {
              int tidx = cylindrical_points.worldToCylindricalIdx(pnt);
              
              if(tidx < max_ind)
              {
                idx = tidx;
              }
            }
            
            if(idx<0)
            {
              idx = cylindrical_points.worldToCanIdx(pnt);
            }
            
            return idx;
        }
        
        
        void insertPoints(ECWrapper& cylindrical_points, const sensor_msgs::LaserScan::ConstPtr &scan_msg, const geometry_msgs::TransformStamped& transform, bool clearing)
        {
            BasicLaserScan2Points converter;
            converter.setScan(scan_msg);
            PointTransformerObject pto(transform);
            const int max_ind = cylindrical_points.getCols();
            
            float* x = (float*)cylindrical_points.getX();
            float* y = (float*)cylindrical_points.getY();
            float* z = (float*)cylindrical_points.getZ();
            
            const int num_inds = (int)scan_msg->ranges.size();
            for(int ind = 0; ind < num_inds; ++ind)
            {
              cv::Point3f point = converter.getPoint(ind);
              if(std::isfinite(point.x))
              {
                cv::Point3f transformed_point = pto.transform(point);
                int idx = getInd(cylindrical_points, transformed_point);
                
                cv::Point3f prev_point(x[idx], y[idx], z[idx]);
                
                float prev_val = -1;
                float new_val = -1;
                if(idx < max_ind)
                {
                  new_val = worldToRangeSquared(transformed_point);
                  prev_val = worldToRangeSquared(prev_point);
                }
                else if(idx >= 0)
                {
                  new_val = worldToCanDepth(transformed_point);
                  prev_val = worldToCanDepth(prev_point);
                }
                else
                {
                    ROS_WARN_STREAM("Invalid index [" << idx << "] for point (" << transformed_point.x << "," << transformed_point.y << "," << transformed_point.z << ")");
                }
                
                
                //if(!(prev_val <= new_val))
                if(new_val < prev_val || clearing)
                {   
                  x[idx] = transformed_point.x;
                  y[idx] = transformed_point.y;
                  z[idx] = transformed_point.z;
                }
              }
            }
        }

      

        LaserScanInserter::LaserScanInserter(tf2_ros::Buffer& buffer, ros::NodeHandle pnh):
          buffer_(buffer),
          pnh_(pnh)
        {}
        
            
        bool LaserScanInserter::init()
        {
            fixed_frame_id_ = "map";
            //TODO: use pips::param_utils
            bool status = pnh_.getParam("fixed_frame_id", fixed_frame_id_);

            return status;
        }
        
        bool LaserScanInserter::init(std::string fixed_frame_id)
        {
            fixed_frame_id_ = fixed_frame_id;
            return true;
        }
        
            
        bool LaserScanInserter::insert(ECWrapper& cylindrical_points, const sensor_msgs::LaserScan::ConstPtr& scan_msg)
        {
            const std_msgs::Header& target_header = cylindrical_points.getHeader();
            const std_msgs::Header& source_header = scan_msg->header;
            
            //Get transform
            geometry_msgs::TransformStamped transform;
            try
            {
                transform = buffer_.lookupTransform(target_header.frame_id, target_header.stamp, source_header.frame_id, source_header.stamp, fixed_frame_id_);
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN_STREAM("Problem finding transform:\n" <<ex.what());
                return false;
            }
            
            const auto& t = transform.transform.translation;
            bool clearing = (t.x == 0 && t.y == 0 && t.z == 0);
            ROS_INFO_STREAM("Laser scan insertion clearing?: " << clearing);
            
            insertPoints(cylindrical_points, scan_msg, transform, clearing);

            return true;
        }

        
    } //end namespace utils
} //end namespace egocylindrical
