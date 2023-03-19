#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

// using namespace Eigen;

namespace uvdar{
    

    struct PointState{
        cv::Point2d point;
        bool ledState; 
        ros::Time insertTime;
        bool computedExtendedSearch = false;
        std::vector<double> x_coeff;
        std::vector<double> y_coeff;
        cv::Point2d ellipse;
        cv::Point2d predicted;
    };

    using seqPointer = std::shared_ptr<std::vector<PointState>>;

    class ExtendedSearch{
        
        private: 
            double decayFactor_;
            int polyOrder_;
            double calcWeightedRMSE(const Eigen::VectorXd, const Eigen::VectorXd, const Eigen::VectorXd);
            std::vector<double> calculateWeightVector(const std::vector<ros::Time>&);

        public:
            ExtendedSearch(double, int);
            ~ExtendedSearch();
            bool checkIfInsideEllipse(PointState&, cv::Point2d&);
            std::pair<std::vector<double>, double> polyReg(const std::vector<double>&, const std::vector<ros::Time>&); 
            // bool doRegression(seqPointer&, std::vector<double>&, std::vector<double>&, std::vector<ros::Time>&);

    };
} // uvdar