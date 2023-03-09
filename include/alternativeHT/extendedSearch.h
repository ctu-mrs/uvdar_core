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
    };

    using seqPointer = std::vector<PointState>*;
    
    struct SeqWithTrajectory{
        seqPointer seq;
        std::vector<double> xCoeff;
        std::vector<double> yCoeff;
    };


    class ExtendedSearch{
        
        private: 
            double decayFactor_;
            int polyOrder_;
            std::vector<double> polyReg(const std::vector<double>&, const std::vector<ros::Time>&); 
        
        public:
            ExtendedSearch(double, int);
            ~ExtendedSearch();
            bool selectPointsForRegressionAndDoRegression(SeqWithTrajectory&);
            std::vector<cv::Point2d> findOrthogonalVectorWithLength(const cv::Point2d, const double);
            float area(const cv::Point2d, const cv::Point2d, const cv::Point2d);
            bool isInside(const cv::Point2d, const cv::Point2d, const cv::Point2d, const cv::Point2d);
    };
} // uvdar