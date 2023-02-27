#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

// using namespace Eigen;

namespace uvdar{
    

    struct PointState{
        cv::Point2d point;
        bool ledState; 
        ros::Time insertTime;
        cv::Point2d firstEdgeTri = cv::Point2d(0,0); 
        cv::Point2d secEdgeTri = cv::Point2d(0,0); 
        cv::Point2d predictedNextPoint = cv::Point2d(0,0);
        double lengthToPredict = 0;
        cv::Point2d debug_diff = cv::Point2d(0,0);
        cv::Point2d debug_gp = cv::Point2d(0,0);
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
            static std::vector<cv::Point2d> findOrthogonalVectorWithLength(const cv::Point2d, const double);
            static float area(const cv::Point2d, const cv::Point2d, const cv::Point2d);
            static bool isInside(const cv::Point2d, const cv::Point2d, const cv::Point2d, const cv::Point2d);
    };
} // uvdar