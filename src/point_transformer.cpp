#include <egocylindrical/ecwrapper.h>
#include <egocylindrical/utils.h>



namespace egocylindrical
{
    
    namespace utils
    {
        /* Inplace transform
        * 
        */
        inline
        void transform_impl(utils::ECStixel& points, const float* const _R, const float* const _T)
        {

//             float* point_ptr = (float*)__builtin_assume_aligned(points, 16);            
//             
            const float* const R = (float*)__builtin_assume_aligned(_R, __BIGGEST_ALIGNMENT__);
            const float* const T = (float*)__builtin_assume_aligned(_T, __BIGGEST_ALIGNMENT__);
            

            const int num_cols = points.getSize();
            std::vector<float>* x = points.getX();
            std::vector<float>* by = points.getBY();
            std::vector<float>* ty = points.getTY();
            std::vector<float>* z = points.getZ();
            
            #pragma GCC ivdep  //https://gcc.gnu.org/onlinedocs/gcc/Loop-Specific-Pragmas.html
            for(size_t p = 0; p < num_cols; ++p)
            {
                float temp_bottom[3];
                float temp_top[3];
                for(int row=0; row < 3; ++row)
                {
                    temp_bottom[row] = R[row*3] * x->at(p) + R[row*3+1] * by->at(p) + R[row*3+2] * z->at(p);
                    temp_bottom[row] += T[row];
                    temp_top[row] = R[row*3] * x->at(p) + R[row*3+1] * ty->at(p) + R[row*3+2] * z->at(p);
                    temp_top[row] += T[row];
                }
                x->at(p) = temp_bottom[0];
                by->at(p) = temp_bottom[1];
                ty->at(p) = temp_top[1];
                z->at(p) = temp_bottom[2];
            }

        }
        
        /*
        inline
        void transform_impl2(const cv::Mat& points, cv::Mat& new_points, const float*  const _R, const float*  const _T)
        {
            float* point_ptr = (float*)__builtin_assume_aligned(points.data, 16);            
            
            const float* const R = (float*)__builtin_assume_aligned(_R, __BIGGEST_ALIGNMENT__);
            const float* const T = (float*)__builtin_assume_aligned(_T, __BIGGEST_ALIGNMENT__);
            
            float* __restrict__ new_point_ptr[] = {n_x,n_y,n_z};
            
            //#pragma omp simd  //aligned(variable[:alignment] [,variable[:alignment]])
            #pragma GCC ivdep
            for(size_t p = 0; p < points.cols; ++p)
            {
                for(int row=0; row < 3; ++row)
                {
                    float temp = 0;
                    for(int col=0; col < 3; ++col)
                    {
                        temp += R[row*3+col] * point_ptr[col][p]; // points.at<float>(col,p);
                    }
                    new_point_ptr[row][p] = temp + T[row];
                }
            }
        }
        */
        
        // TODO: This functionality could be moved into a tf2_ros implementation
        void transformPoints(utils::ECStixel& points, const geometry_msgs::TransformStamped& trans)
        {
            
  
            tf::Quaternion rotationQuaternion = tf::Quaternion(trans.transform.rotation.x,
                                trans.transform.rotation.y,
                                trans.transform.rotation.z,
                                trans.transform.rotation.w);


            tf::Matrix3x3 tempRotationMatrix = tf::Matrix3x3(rotationQuaternion);


            ROS_DEBUG("Getting Rotation Matrix");
            float rotationArray[9]  __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
//             float rotationArray[9];
            rotationArray[0] = (float) tempRotationMatrix[0].getX();
            rotationArray[1] = (float) tempRotationMatrix[0].getY();
            rotationArray[2] = (float) tempRotationMatrix[0].getZ();
            rotationArray[3] = (float) tempRotationMatrix[1].getX();
            rotationArray[4] = (float) tempRotationMatrix[1].getY();
            rotationArray[5] = (float) tempRotationMatrix[1].getZ();
            rotationArray[6] = (float) tempRotationMatrix[2].getX();
            rotationArray[7] = (float) tempRotationMatrix[2].getY();
            rotationArray[8] = (float) tempRotationMatrix[2].getZ();
            //cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
            
            float translationArray[3] __attribute__ ((aligned (__BIGGEST_ALIGNMENT__)));
//             float translationArray[3];
            translationArray[0] = trans.transform.translation.x;
            translationArray[1] = trans.transform.translation.y;
            translationArray[2] = trans.transform.translation.z;
            
//             cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, &rotationArray[0]);
//             cv::Mat translationMatrix = cv::Mat(3, 1, CV_32F, &translationArray[0]);
            
            transform_impl(points, rotationArray, translationArray);

        }
        
    }
    
}
