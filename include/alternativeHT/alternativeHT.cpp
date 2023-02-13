#include "alternativeHT.h"

#include <bits/stdc++.h>

using namespace uvdar;

alternativeHT::alternativeHT(){
}

void alternativeHT::setDebugFlags(bool debug, bool visual_debug){
    debug_ = debug;
    visual_debug_ = visual_debug;
}

void alternativeHT::setSequences(std::vector<std::vector<bool>> i_sequences){
  
    originalSequences_ = i_sequences;
    matcher_ = std::make_unique<SignalMatcher>(originalSequences_);
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    
    std::vector<PointState> currentFrame;
    for ( auto pointWithTimeStamp : ptsMsg->points) {
        PointState p;
        p.point = cv::Point(pointWithTimeStamp.x, pointWithTimeStamp.y);
        p.ledState = true; // every existent point is "on"
        p.insertTime = ptsMsg->stamp;
        cv::Point2d leftUpperCorner, rightDownCorner;
        leftUpperCorner.x = p.point.x - boundingBox_x_Size_; 
        leftUpperCorner.y = p.point.y - boundingBox_y_Size_;
        rightDownCorner.x = p.point.x + boundingBox_x_Size_;
        rightDownCorner.y = p.point.y + boundingBox_y_Size_;
        if (p.bbLeftUp.x < 0) p.bbLeftUp.x = 0;
        if (p.bbLeftUp.y < 0) p.bbLeftUp.y = 0;
        p.bbLeftUp = leftUpperCorner;
        p.bbRightDown = rightDownCorner;
        currentFrame.push_back(p);
    }

    findClosestPixelAndInsert(currentFrame);
    // for (const auto k : generatedSequences_){
    //     std::vector<bool> p;
    //     for (auto r : k){
    //         if(r.ledState) p.push_back(true);
    //         else p.push_back(false);
    //     }
    //     printVectorIfNotEmpty(p, "before");
    // }
    inserVPIfNoNewPointArrived(currentFrame);
// std::cout << "---------------------------------------------\n";
    cleanPotentialBuffer();  // TODO: NOT WORKING!!!!!
    for (const auto k : generatedSequences_){
        std::vector<bool> p;
        for (auto r : k){
            if(r.ledState) p.push_back(true);
            else p.push_back(false);
        }
        // printVectorIfNotEmpty(p, "after");
        // for( auto p : k){
        //     std::cout << p.insertTime << ", ";
        // }
        // std::cout << std::endl;
    }
}

void alternativeHT::findClosestPixelAndInsert(std::vector<PointState> & currentFrame) {   
    
    // std::list<std::vector<PointState>*> pGenSequence;
    std::vector<std::vector<PointState>*> pGenSequence;
    {
       std::scoped_lock lock(mutex_generatedSequences_);
        for(auto & seq : generatedSequences_){
            pGenSequence.push_back(&seq);
        }

    }



    std::vector<PointState> noNN;
    for(auto & currPoint : currentFrame){
        std::scoped_lock lock(mutex_generatedSequences_);
        bool nearestNeighbor = false;
        for(auto seq = pGenSequence.begin(); seq != pGenSequence.end(); ++seq){
            PointState lastInserted = (*seq)->end()[-1];
            cv::Point2d diff = computeXYDiff(currPoint.point, lastInserted.point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;
                insertPointToSequence(*(*seq), currPoint);    
                pGenSequence.erase(seq);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            noNN.push_back(currPoint);
// // here was before ther "stasrt new sequence"
            std::vector<PointState> vect;
            vect.push_back(currPoint);
            generatedSequences_.push_back(vect);
        }
    }

    // if((int)noNN.size() != 0 ){
    //     std::cout << "HEY\n"; 
    //     checkBoundingBoxIntersection(noNN, pGenSequence);
    // }
   
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertPointToSequence(std::vector<PointState> & sequence, const PointState & signal){
        sequence.push_back(signal);            
        if(sequence.size() > originalSequences_[0].size()){
            sequence.erase(sequence.begin());
        }
}

 // for all sequences where no correspondence was found insert VP
void alternativeHT::inserVPIfNoNewPointArrived(std::vector<PointState> & currentFrame){
    std::scoped_lock lock(mutex_generatedSequences_);
    for(auto & sequence : generatedSequences_){
        
        // if the two latest inserted led-states are off, don't insert a third "off"-state 
        if(sequence.size() >= 2){
            bool lastLEDstate = sequence.end()[-1].ledState;
            bool secondLastLEDstate = sequence.end()[-2].ledState;
            if(!lastLEDstate && !secondLastLEDstate){
                continue;
            }
        }
        
        if(currentFrame.size() != 0){
            ros::Time lastInsert = currentFrame[0].insertTime;
            if(lastInsert != sequence.end()[-1].insertTime){
                auto p = sequence.end()[-1];
                p.ledState = false; 
                p.insertTime = ros::Time::now();
                insertPointToSequence(sequence, p);
            }
        }else{
            auto p = sequence.end()[-1];
            p.ledState = false; 
            p.insertTime = ros::Time::now();;
            insertPointToSequence(sequence, p);
        }
    }
}

void uvdar::alternativeHT::checkBoundingBoxIntersection(std::vector<PointState> & noNNCurrentFrame, std::vector<std::vector<PointState>*> & sequencesNoInsert){


    std::vector<std::vector<PointState>> allCombinationsFrame;
    permute(noNNCurrentFrame, 0, (int)(noNNCurrentFrame.size() - 1), allCombinationsFrame);

    for(auto perm : allCombinationsFrame){
        computeHypothesisSets(perm, sequencesNoInsert);
    }
    





        // std::vector<PointState> vect;
        // vect.push_back(pts);
        // generatedSequences_.push_back(vect);
    
}

bool uvdar::alternativeHT::onSegment(PointState p, PointState q, PointState r)
{
	if (q.point.x <= std::max(p.point.x, r.point.x) && q.point.x >= std::min(p.point.x, r.point.x) &&
		q.point.y <= std::max(p.point.y, r.point.y) && q.point.y >= std::min(p.point.y, r.point.y))
	return true;

	return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are collinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int uvdar::alternativeHT::orientation(PointState p, PointState q, PointState r)
{
	// See https://www.geeksforgeeks.org/orientation-3-ordered-points/
    int val = (q.point.y - p.point.y) * (r.point.x - q.point.x) -
              (q.point.x - p.point.x) * (r.point.y - q.point.y);
  
	if (val == 0) return 0; // collinear

	return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool uvdar::alternativeHT::doIntersect(PointState p1, PointState q1, PointState p2, PointState q2)
{
	// Find the four orientations needed for general and
	// special cases
	int o1 = orientation(p1, q1, p2);
	int o2 = orientation(p1, q1, q2);
	int o3 = orientation(p2, q2, p1);
	int o4 = orientation(p2, q2, q1);

	// General case
	if (o1 != o2 && o3 != o4)
		return true;

	// Special Cases
	// p1, q1 and p2 are collinear and p2 lies on segment p1q1
	if (o1 == 0 && onSegment(p1, p2, q1)) return true;

	// p1, q1 and q2 are collinear and q2 lies on segment p1q1
	if (o2 == 0 && onSegment(p1, q2, q1)) return true;

	// p2, q2 and p1 are collinear and p1 lies on segment p2q2
	if (o3 == 0 && onSegment(p2, p1, q2)) return true;

	// p2, q2 and q1 are collinear and q1 lies on segment p2q2
	if (o4 == 0 && onSegment(p2, q1, q2)) return true;

	return false; // Doesn't fall in any of the above cases
}

void permute( std::vector<PointState> a, int l, int r, std::vector<std::vector<PointState>> & result) {
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

//TODO: rename
void uvdar::alternativeHT::computeHypothesisSets(const std::vector<PointState> & hypothesisSet, std::vector<std::vector<PointState>*> sequencesNoInsert){

}

void calcVariance(std::vector<PointState> & distVector) {

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

bool uvdar::alternativeHT::checkValidWithPotentialNewPoint(const std::vector<PointState> & seq){

    if(seq.size() <= 1){
        return true;
    }
    if ((seq.end()[-1].ledState && seq.end()[-2].ledState)){
        return false;
    }

    return true; 
}

// return true, if bounding boxes hit 
bool uvdar::alternativeHT::bbIntersect(const PointState & point1, const PointState & point2){
    
    if( (point1.bbRightDown.x >= point2.bbLeftUp.x && point2.bbRightDown.x >= point1.bbLeftUp.x ) && (point1.bbRightDown.y >= point2.bbLeftUp.y && point2.bbRightDown.y >= point1.bbLeftUp.y)){
        return true;
    }
    return false; 
}

//TODO: Change to ICP
std::vector<PointState>* uvdar::alternativeHT::findClosestWithinSelectedBB(std::vector<std::vector<PointState>*> boxMatchPoints, const PointState queryPoint){

    cv::Point2d min;

    std::vector<PointState>* selected = nullptr; 
    // guarantee that selected will be assigned - otherwise this function shoudln't be called 
    min.x = boundingBox_x_Size_*3;
    min.y = boundingBox_y_Size_*3;
    for(auto p : boxMatchPoints){
        auto matchedPoint = p->end()[-1];
        cv::Point2d diff = computeXYDiff(matchedPoint.point, queryPoint.point);

        if( (diff.x <= min.x && diff.y <= min.y)){
            selected = p;
            min.x = diff.x; 
            min.y = diff.y;
        }
    }
    return selected;
}

void alternativeHT::cleanPotentialBuffer(){

//TODO: eventually not needed - correction in Moving Avg part
    double timeThreeFrames = (1/60.0) * 4; 

    std::scoped_lock lock(mutex_generatedSequences_);
    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        }

        if(it->size() >= 3){
            bool first  = it->end()[-1].ledState;
            bool second = it->end()[-2].ledState;
            bool third  = it->end()[-3].ledState;
            // if(first && second && third && forth){
            //     ROS_ERROR("all true");
            //     it = generatedSequences_.erase(it);
            //     continue;
            // }
            if(!first && !second && !third){
                ROS_ERROR("all false");
                it = generatedSequences_.erase(it);
                continue;
            }
        }
        
        
        
        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        // std::cout << "time last inserted" << timeDiffLastInsert << "last insedrt" << insertTime; 
        if (timeDiffLastInsert > timeThreeFrames){ //&& i != generatedSequences_.end()){
            ROS_ERROR("TIME");
            it = generatedSequences_.erase(it);
            continue;
        }

    }
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

    for (auto sequence : generatedSequences_){
        std::vector<bool> ledStates;
        for (auto point : sequence){
            ledStates.push_back(point.ledState);
        }
        int id = findSequenceMatch(ledStates);
        retrievedSignals.push_back(std::make_pair(sequence.end()[-1], id));
    }

    return retrievedSignals;
}

int alternativeHT::findSequenceMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    int id = matcher_->matchSignalWithCrossCorr(sequence);

    return id;
}

alternativeHT::~alternativeHT() {
    // delete generatedSequences_;
}