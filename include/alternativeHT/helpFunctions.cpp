#include "helpFunctions.h"


namespace uvdar{

    HelpFunctions::HelpFunctions(){

    }

    HelpFunctions::~HelpFunctions(){

    }



    void HelpFunctions::permute( std::vector<PointState> a, int l, int r, std::vector<std::vector<PointState>> & result) {
        // Base case
        if (l == r)
        {
            result.push_back(a);
        }else {
            // Permutations made
            for (int i = l; i <= r; i++) {
            
                // Swapping done
                std::swap(a[l], a[i]);
    
                // Recursion called
                permute(a, l + 1, r, result);
    
                // backtrack
                std::swap(a[l], a[i]);
            }
        }
    }

    void HelpFunctions::calcVariance(std::vector<PointState> & distVector) {

        double sizeVector = (double)distVector.size();
        double xAvg = 0.0;
        double yAvg = 0.0;  

        // std::cout << "Vector " <<std::endl;
        // for(auto k : distVector){
        //     std::cout << "{" << k.x << "," << k.y << "}";
        // }
        // std::cout << std::endl;

        // compute average
        for( const auto vect : distVector){
            xAvg = xAvg + vect.point.x; 
            yAvg = yAvg + vect.point.y;
        }

        xAvg = xAvg/sizeVector;
        yAvg = yAvg/sizeVector;
        // std::cout << "avg " << xAvg << ", " << yAvg << "\n";

        PointState variance;
        std::vector<PointState> sum;
        for(const auto vect : distVector){
            variance.point.x = ((vect.point.x - xAvg)*(vect.point.x - xAvg));
            variance.point.y = ((vect.point.y - yAvg)*(vect.point.y - yAvg));  
            // std::cout << "var x" << variance.x << ", " << variance.y << std::endl;
            sum.push_back(variance);
        }

        PointState p; 
        p.point.x = 0;
        p.point.y = 0; 
        for(auto k : sum){
            p.point.x = p.point.x + k.point.x; 
            p.point.y = p.point.y + k.point.y;
        }

        p.point.x = p.point.x / (double)sizeVector;
        p.point.y = p.point.y / (double)sizeVector;

        // std::cout << "Variance x " << p.x << " Variance y " << p.y <<std::endl;

    }

    // void HelpFunctions::polynomialRegression(const std::vector<PointState> sequence){
    //     std::vector<PointState> selectedPts;
    //     this->selectLastFourOnLEDs(sequence, selectedPts);
        
    // }

    void HelpFunctions::selectLastNDataPoints(const std::vector<PointState>& seq, std::vector<PointState>& selectedPoints, int numerOfDataPoints ){
    //     std::cout << "HERE\n";
        if((int)seq.size() < numerOfDataPoints){
            ROS_ERROR("[UVDAR_BP_Tim]: The wanted number for the polynomial regression is higher than the sequence lenght!");
            return;
        }
        for(auto itSeq = seq.end(); itSeq != seq.begin(); --itSeq){
            if(itSeq->ledState){
               selectedPoints.push_back(*itSeq);
            }
            // if((int)selectedPoints.size() == numerOfDataPoints){
            //     return;
            // }
        }

    }

    std::pair<std::vector<double>, std::vector<double>> HelpFunctions::prepareForPolyReg(std::vector<PointState>* sequence, const int numberOfDataPoints, const int order){

        std::vector<PointState> selectedPts;
        HelpFunctions::selectLastNDataPoints(*sequence, selectedPts, numberOfDataPoints);

        std::vector<double> x,y;
        std::vector<ros::Time> time;
        for(const auto point : selectedPts){
            x.push_back(point.point.x);
            y.push_back(point.point.y);
            time.push_back(point.insertTime);
        }

        std::vector<double> coeffX = polyReg(x, time, order); 
        std::vector<double> coeffY = polyReg(y, time, order);
        std::pair<std::vector<double>, std::vector<double>> polyRegression = std::make_pair(coeffX, coeffY);

    }

    std::vector<double> HelpFunctions::polyReg(const std::vector<double>& pixelCoordinate, const std::vector<ros::Time>& time, const int order){


        Eigen::MatrixXd DesignMat(time.size(), order + 1);
	    Eigen::VectorXd pixelMat = Eigen::VectorXd::Map(&pixelCoordinate.front(), pixelCoordinate.size());
        Eigen::VectorXd result(order+1);

        // fill the Design matrix
	    for(int i = 0 ; i < (int)time.size(); ++i){
	    	for(int j = 0; j < order + 1; ++j){
	    		DesignMat(i, j) = pow(time[i].toSec(), j);
	    	}
	    }
	    // Solve for linear least square fit
	    result = DesignMat.householderQr().solve(pixelMat);
        std::vector<double> coeff;
        for(int i = 0; i < result.size(); ++i){
            coeff.push_back(result[i]);
        }
        return coeff;
    }




} // uvdar