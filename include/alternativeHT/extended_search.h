#include <ros/console.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <bits/stdc++.h>


namespace uvdar{
    
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
            std::vector<double> calcPredictionVector(const std::vector<double>&, const std::vector<double>&);
            bool checkIfInsideEllipse(const cv::Point2d&, const cv::Point2d&, const cv::Point2d&);
            std::pair<std::vector<double>, Eigen::VectorXd> polyReg(const std::vector<double>&, const std::vector<double>&, const std::vector<double>& weights);
            double standardErrorPrediction(Eigen::VectorXd&, const std::vector<double>&, std::vector<double>&, const double&, const double&);
    };
} // uvdar