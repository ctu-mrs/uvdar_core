#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

// using namespace Eigen;

namespace uvdar{
    

    struct PointState{
        cv::Point2d point;
        bool ledState; 
        ros::Time insertTime;
    };

    using seqPointer = std::vector<PointState>*;
    
    struct SeqWithTrajectory{
        seqPointer seq;
        std::vector<double> xCoeff;
        std::vector<double> yCoeff;
        cv::Point2d rmse_poly_reg;
        cv::Point2d predicted;
    };


    class ExtendedSearch{
        
        private: 
            double decayFactor_;
            int polyOrder_;
            std::vector<double> polyReg(const std::vector<double>&, const std::vector<ros::Time>&); 
            double calcRMSE(const std::vector<double>&, const std::vector<ros::Time>&, const std::vector<double>&);
        
        public:
            ExtendedSearch(double, int);
            ~ExtendedSearch();
            double checkIfInsideEllipse(SeqWithTrajectory&, cv::Point2d&);
            bool selectPointsForRegressionAndDoRegression(SeqWithTrajectory&);

    };
} // uvdar