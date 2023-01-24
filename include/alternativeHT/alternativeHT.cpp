#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
    initBuffer();
}

void alternativeHT::initBuffer(){
    std::scoped_lock lock(mutex_buffer);
    for (int i = 0; i < buffer_size_; i++){
        std::vector<PointState> dummyVector;
        buffer.push_back(dummyVector);
    }
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
    {   
        std::scoped_lock lock(mutex_buffer);
        std::vector<mrs_msgs::Point2DWithFloat> point3D(std::begin(ptsMsg->points), std::end(ptsMsg->points));
        std::vector<PointState> points; 
        for ( size_t i = 0; i < point3D.size(); i++ ) {
            PointState p;
            p.point = cv::Point(point3D[i].x, point3D[i].y);
            p.index = -1;
            p.ledState = true; // every existent point is "on"
            p.insertTime = ptsMsg->stamp;
            points.push_back(p);
        }

        if((int)buffer.size() < buffer_size_){
            buffer.push_back(points);
        }else{
            buffer.erase(buffer.begin());
            buffer.push_back(points);
        }

        if((int)buffer.size() < 2){
            return;
        }
    }
    findClosestAndLEDState();
    cleanPotentialBuffer();
    checkIfThreeConsecutiveZeros();
}

void alternativeHT::checkIfThreeConsecutiveZeros(){
    
    if (first_call_){
        return;
    }
    std::scoped_lock lock(mutex_buffer);
    std::vector<PointState> & ptsCurr= buffer.end()[-1];
    std::vector<PointState> & ptsSec = buffer.end()[-2];
    std::vector<PointState> & ptsThird = buffer.end()[-3];

    for (int i = 0; i < ptsCurr.size(); i++){
        int indexCurr = ptsCurr[i].index;
        bool boolCurr = ptsCurr[i].ledState; 
        for(int k = 0; k < ptsSec.size(); k++){
            int indexSec = ptsSec[k].index;
            bool boolSec = ptsSec[k].ledState;
            for (int j =0; j<ptsThird.size(); j++){
                int indexThird = ptsThird[j].index;
                bool boolThird = ptsThird[j].ledState; 
                if (indexThird == indexSec && indexSec == indexCurr){
                    if (boolThird == false && boolSec == false && boolCurr == false){
                        ROS_ERROR("At least here?!");
                        ptsCurr.erase(ptsCurr.begin()+i);
                        ptsThird.erase(ptsThird.begin()+j);
                        ptsSec.erase(ptsSec.begin()+k);
                    }
                }
            }

        }
    }
}

void alternativeHT::findClosestAndLEDState() {   
    
    std::scoped_lock lock(mutex_buffer);
    
    std::vector<PointState> & ptsCurrentImg = buffer.end()[-1]; // newest points;
    std::vector<PointState> & ptsPrevImg    = buffer.end()[-2]; // second newest points
    std::cout << "=============================" <<std::endl; 
    for(int iPrev = 0;  iPrev < (int)ptsPrevImg.size(); iPrev++){
        bool nearestNeighbor = false;
        for(int iCurr = 0; iCurr < (int)ptsCurrentImg.size(); iCurr++){
            cv::Point2d diff = computeXYDiff(ptsCurrentImg[iCurr].point, ptsPrevImg[iPrev].point);
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                nearestNeighbor = true;           
                if(ptsPrevImg[iPrev].index == -1){
                    ptsPrevImg[iPrev].index = 1;
                    ptsCurrentImg[iCurr].index = 1; 
                    sequenceWithPoint newSequence;
                    newSequence.lastInsertedPoint = ptsCurrentImg[iCurr];
                    newSequence.seq.push_back(ptsPrevImg[iPrev].ledState);
                    newSequence.seq.push_back(ptsPrevImg[iPrev].ledState);
                    foundSequences_.insert(foundSequences_.end(),newSequence);
                    std::cout << "The First size " << foundSequences_.size() << std::endl;
                }else{
                    ptsCurrentImg[iCurr].index = 1;
                    for ( auto & it : foundSequences_){
                        if(equalElements(it.lastInsertedPoint, ptsPrevImg[iPrev])){
                            it.seq.push_back(ptsCurrentImg[iCurr].ledState); 
                            it.lastInsertedPoint = ptsCurrentImg[iCurr]; 
                        }else{
                            ROS_ERROR("I fucked up"); 
                        }
                    }
                    std::cout << "The Second size " << foundSequences_.size() << std::endl;
                }
                break;
            }else{
                std::cout << "The point vals are " << diff.x << ", " << diff.y << std::endl; 
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            if (ptsPrevImg[iPrev].index == -1){
                ptsPrevImg[iPrev].index = 1;// randomInt(0);//size;
                sequenceWithPoint newSequence;//std::make_shared<sequenceWithPoint>(); 
                newSequence.lastInsertedPoint = ptsPrevImg[iPrev];
                newSequence.seq.push_back(ptsPrevImg[iPrev].ledState);
                foundSequences_.insert(foundSequences_.end(),newSequence);
            }

            std::cout << " INSERT VP \n"; 
            insertVirtualPoint(ptsPrevImg[iPrev]);

            for ( auto & it : foundSequences_){
                if(equalElements(it.lastInsertedPoint,ptsPrevImg[iPrev])){
                    it.seq.push_back(ptsCurrentImg.end()[-1].ledState); 
                    it.lastInsertedPoint = ptsCurrentImg.end()[-1]; 
                }else{
                ROS_ERROR("I fucked up"); 
                }
            }
        }
    }
    std::cout << "The sequence size " << foundSequences_.size() << std::endl;
}

bool uvdar::alternativeHT::equalElements(PointState p1, PointState p2 ){
    if (p1.point.x == p2.point.x && p1.point.y == p2.point.y && p1.insertTime == p2.insertTime){
        std::cout << "Here "; 
        return true;
    }
    return false; 
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertVirtualPoint(const PointState signalPrevFrame){   

    PointState offState;
    offState.point = signalPrevFrame.point;
    offState.index = signalPrevFrame.index;
    offState.ledState = false;
    offState.insertTime = ros::Time::now();
    buffer.end()[-1].push_back(offState);
}

void alternativeHT::cleanPotentialBuffer(){
    double timeThreeFrames = (1/60.0) * 4; 

    std::scoped_lock lock(mutex_generatedSequences_);

    std::map<int, sequenceWithPoint> b = generatedSequences_;

    auto k = foundSequences_.begin();
    while (k != foundSequences_.end()){
        double insertTime = k->lastInsertedPoint.insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        if (timeDiffLastInsert > timeThreeFrames){
            ROS_WARN("DELETING"); 
            k = foundSequences_.erase(k);
        
            continue;
        }
        k++;
    }

    std::cout <<"Size of sequences" << generatedSequences_.size() << "\n";
    for (auto k : generatedSequences_) { 
        printVectorIfNotEmpty(k.second.seq, "Sequence");
    }
}


int alternativeHT::findMatch(std::vector<bool> sequence){

    for ( int i = 2; i < (int)sequence.size(); i++) {
        if (sequence[i] == false && sequence[i-1] == false && sequence[i-2] == false){
            return -2;
        }
    }
    int id = matcher_->matchSignalWithCrossCorr(sequence);

    return id;
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

        

//     std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;

    for (auto k : foundSequences_){
        int id = findMatch(k.seq);
        PointState originPoint = k.lastInsertedPoint;
        retrievedSignals.push_back(std::make_pair(originPoint, id));
        }
    return retrievedSignals;
}

alternativeHT::~alternativeHT() {
}