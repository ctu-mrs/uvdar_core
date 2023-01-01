#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <mrs_msgs/Point2DWithFloat.h>

using vectPoint3D = std::vector<mrs_msgs::Point2DWithFloat>;

namespace uvdar
{

    class Fast_Buffer{
        using pair = std::pair<vectPoint3D, std::vector<bool>>;
        private:
            pair frame_pts_with_state_bool;
        public:
            void set_frame_pts(vectPoint3D pts){
                frame_pts_with_state_bool.first = pts;
            }
            void set_bools(std::vector<bool> state){
                frame_pts_with_state_bool.second = state;
            }
            
            vectPoint3D & get_pts_reference(){ 
                return frame_pts_with_state_bool.first; 
            }
           
           
            pair get_frame_pts_with_state_bool(){
                return frame_pts_with_state_bool;
            }
            
    };


    class alternativeHT
    {
    private:
        int buffer_size_;
        bool first_call_ = true; 
        
        const int max_pixel_shift_x_ = 3;
        const int max_pixel_shift_y_ = 3;

        // buffer size, vector with Points
        std::vector<vectPoint3D> buffer_with_frame_points;
        std::vector<Fast_Buffer> fast_buffer_;



    public:
        alternativeHT( int i_buffer_size );
        ~alternativeHT();
        void setFirstCallBool(bool i_first_call){
            first_call_ = i_first_call;
        }

        void processBuffer(vectPoint3D &, const int );
        bool checkInsertVP(vectPoint3D &, vectPoint3D &);
        bool bothFramesEmpty(vectPoint3D , vectPoint3D );
        void insertEmptyPoint(vectPoint3D &, const mrs_msgs::Point2DWithFloat);

        void findClosestAndLEDState(vectPoint3D &, vectPoint3D &);
        void cleanUpBuffer(const int);



    };


    

    
} // namespace uvdar

