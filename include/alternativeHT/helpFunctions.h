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

    class HelpFunctions{
        private: 
            HelpFunctions();
            ~HelpFunctions();
            static void selectLastNDataPoints(std::vector<PointState>, std::vector<PointState>, int);
        public:
            static void permute( std::vector<PointState>, int, int, std::vector<std::vector<PointState>>&);
            static void calcVariance(std::vector<PointState>&);
            static std::pair<std::vector<PointState>*, Eigen::VectorXd> polynomialRegression(std::vector<PointState>*, int);
    };

} // uvdar