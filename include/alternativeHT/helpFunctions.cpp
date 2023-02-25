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


// returns always true!
    bool HelpFunctions::prepareForPolyReg(SeqWithTrajectory & prediction, const int order){
        
        std::vector<double> x,y;
        std::vector<ros::Time> time;

        for(const auto point : *(prediction.seq)){
            if(point.ledState){
                x.push_back(point.point.x);
                y.push_back(point.point.y);
                time.push_back(point.insertTime);
            }
        }

        // if(x.size() < 4 || y.size() < 4) return false;
        // std::cout << "the x vals : " <<  std::endl;
        prediction.xCoeff = polyReg(x, time, order);
        // std::cout << "the y vals : " <<  std::endl; 
        prediction.yCoeff = polyReg(y, time, order);

        return true;
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
        // for(int k = 0; k < pixelCoordinate.size(); ++k){
            // std::cout << pixelCoordinate[k] << ", " << time[k] << " ~ ";
        // }
        // std::cout << "\n";
        // std::cout << "The coeff \n";
        // for (const auto k : coeff){
        //     std::cout << k << ", "; 
        // }
        // std::cout << std::endl;
        return coeff;
    }


    std::vector<cv::Point2d> HelpFunctions::findOrthogonalVectorWithLength(const cv::Point2d vect, const double len){

        cv::Point2d orthoFirst;
        cv::Point2d orthoSecond; 
        // second solution also needed!

        if(len < 0.001){
            std::cout << "TOO SHORT " << std::endl;
        }

        orthoFirst.y =  sqrt( pow(len,2) / (pow((vect.y/vect.x),1)) );  
        orthoFirst.x = - ( orthoFirst.y * vect.y ) / vect.x;

        orthoSecond.y =  -sqrt( pow(len,2) / (pow((vect.y/vect.x),1)) );  
        orthoSecond.x = - ( orthoSecond.y * vect.y ) / vect.x;

        std::vector<cv::Point2d> solutions;
        solutions.push_back(orthoFirst);
        solutions.push_back(orthoSecond);
        return solutions;
    }


    float HelpFunctions::area(const cv::Point2d p1, const cv::Point2d p2, const cv::Point2d p3){
       return abs((p1.x*(p2.y-p3.y) + p2.x*(p3.y-p1.y)+ p3.x*(p1.y-p2.y))/2.0);
    }
    
    bool HelpFunctions::isInside(const cv::Point2d p1, const cv::Point2d p2, const cv::Point2d p3, const cv::Point2d query){  
       /* Calculate area of triangle ABC */
       float A = HelpFunctions::area (p1, p2, p3);
    
       /* Calculate area of triangle PBC */ 
       float A1 = HelpFunctions::area (query, p2, p3);
    
       /* Calculate area of triangle PAC */ 
       float A2 = HelpFunctions::area (p1, query, p3);
    
       /* Calculate area of triangle PAB */  
       float A3 = HelpFunctions::area (p1, p2, query);

       /* Check if sum of A1, A2 and A3 is same as A */
       return (A == A1 + A2 + A3);
    }

} // uvdar