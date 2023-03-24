#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <boost/math/distributions/students_t.hpp>
#include <bits/stdc++.h>



namespace uvdar{

    struct PredictionStatistics{
        std::vector<double> coeff;
        Eigen::VectorXd predicted_vals_past; 
        int used_poly_order;
        double mean;
        double predicted_coordinate = -1;
        double ellipse_val = -1;
    };
    
    class ExtendedSearch{
        
        private: 
            double decay_factor_;
            int default_poly_order_;
            // double weightedSumSqauredEstimate(const Eigen::VectorXd, const Eigen::VectorXd, const Eigen::VectorXd);
            double calcWMSE(const Eigen::VectorXd&, const std::vector<double>&, const std::vector<double>&);

        public:
            ExtendedSearch(double, int);
            ~ExtendedSearch();
            std::vector<double> calculateWeightVector(const std::vector<double>&);
            double calcWeightedMean(const std::vector<double>&, const std::vector<double>&);
            double calcWSTD(const std::vector<double>&, const std::vector<double>&, const double&);
            bool checkIfInsideEllipse(const cv::Point2d&, const cv::Point2d&, const cv::Point2d&);
            PredictionStatistics polyReg(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&);
            double confidenceInterval(const PredictionStatistics&, const std::vector<double>&, const std::vector<double>, const int&);
    };
} // uvdar