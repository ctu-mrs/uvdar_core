#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

// using namespace Eigen;

namespace uvdar{
    

    struct PointState{
        cv::Point2d point;
        bool ledState; 
        ros::Time insertTime;
        cv::Point2d bbLeftUp; 
        cv::Point2d bbRightDown;
    };

    using seqPointer = std::vector<PointState>*;
    
    struct SeqWithTrajectory{
        seqPointer seq;
        std::vector<double> xCoeff;
        std::vector<double> yCoeff;
    };


    class HelpFunctions{
        private: 
            HelpFunctions();
            ~HelpFunctions();
            // static bool selectLastNDataPoints(const std::vector<PointState>&, std::vector<PointState>&, int);
            static std::vector<double> polyReg(const std::vector<double>&, const std::vector<ros::Time>&, const int); 
        public:
            static void permute( std::vector<PointState>, int, int, std::vector<std::vector<PointState>>&);
            static void calcVariance(std::vector<PointState>&);
            static bool prepareForPolyReg(SeqWithTrajectory&, const int);
            
    };

} // uvdar