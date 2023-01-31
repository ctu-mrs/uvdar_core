#include "alternativeHT.h"

using namespace uvdar;

alternativeHT::alternativeHT( int i_buffer_size ){
    
    buffer_size_ = i_buffer_size;
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
    initBuffer();
    
}

void alternativeHT::processBuffer(const mrs_msgs::ImagePointsWithFloatStampedConstPtr ptsMsg) {
    {   
        std::scoped_lock lock(mutex_buffer);
        std::vector<mrs_msgs::Point2DWithFloat> point3D(std::begin(ptsMsg->points), std::end(ptsMsg->points));
        std::vector<PointState> points; 
        for ( size_t i = 0; i < point3D.size(); i++ ) {
            PointState p;
            p.point = cv::Point(point3D[i].x, point3D[i].y);
            p.ledState = true; // every existent point is "on"
            p.insertTime = ptsMsg->stamp;
            cv::Point2d leftUpperCorner, rightDownCorner;
            leftUpperCorner.x = p.point.x - boundingBox_x_Size_; 
            leftUpperCorner.y = p.point.y - boundingBox_y_Size_;
            if (p.bbLeftUp.x < 0) p.bbLeftUp.x = 0;
            if (p.bbLeftUp.y < 0) p.bbLeftUp.y = 0;
            p.bbLeftUp = leftUpperCorner;
            p.bbRightDown = rightDownCorner;
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


        // std::cout << "The buffer is: " << std::endl;
        // for (const auto k : buffer ){
            // for (const auto r : k){
                // if (r.ledState){ std::cout<< "1,";}
                // else std::cout << "0,";
            // } std::cout << std::endl;
        // }
    }

    findClosestAndLEDState();
    cleanPotentialBuffer(); 
    // checkIfThreeConsecutiveZeros();
}

void alternativeHT::findClosestAndLEDState() {   
    
    std::scoped_lock lock(mutex_buffer);
    
    std::vector<PointState> & ptsCurrentImg = buffer.end()[-1]; // newest points;
    std::vector<PointState> & ptsPrevImg    = buffer.end()[-2]; // second newest points
    
    for(int iPrev = 0;  iPrev < (int)ptsPrevImg.size(); iPrev++){
        bool nearestNeighbor = false;
        for(int iCurr = 0; iCurr < (int)ptsCurrentImg.size(); iCurr++){
            cv::Point2d diff = computeXYDiff(ptsCurrentImg[iCurr].point, ptsPrevImg[iPrev].point);
            // std::cout << "The diff are " << diff.x << ", " << diff.y << std::endl; 
            if(diff.x <= max_pixel_shift_x_ && diff.y <= max_pixel_shift_y_){
                std::scoped_lock lock(mutex_generatedSequences_);
                nearestNeighbor = true;
                if (!ptsPrevImg[iPrev].insertedToSeq){
                    insertPointToSequence(ptsPrevImg[iPrev], ptsPrevImg[iPrev]);
                }
                insertPointToSequence(ptsCurrentImg[iCurr], ptsPrevImg[iPrev]);
                break;
            }else{
                nearestNeighbor = false;
            }
        }
        if(nearestNeighbor == false){
            std::scoped_lock lock(mutex_generatedSequences_);
            if (!ptsPrevImg[iPrev].insertedToSeq){
                ROS_ERROR("2---NOT INSERTED");
                // insertPointToSequence(ptsPrevImg[iPrev], ptsPrevImg[iPrev]);
            }
            insertVirtualPoint(ptsPrevImg[iPrev]);
            insertPointToSequence(ptsCurrentImg.end()[-1], ptsPrevImg[iPrev]);
        }
    }
}

cv::Point2d uvdar::alternativeHT::computeXYDiff(const cv::Point2d first, const cv::Point2d second){
    
    cv::Point2d difference; 
    difference.x = std::abs(first.x - second.x);
    difference.y = std::abs(first.y - second.y);
    return difference;
}

void alternativeHT::insertVirtualPoint(const PointState signalPrevFrame){   
    PointState offState;
    offState.insertedToSeq = false;
    offState.point = signalPrevFrame.point;
    offState.ledState = false;
    offState.insertTime = ros::Time::now();
    buffer.end()[-1].push_back(offState);
}

void alternativeHT::insertPointToSequence(PointState & signal, PointState prev){
    
    if(generatedSequences_.size() == 0 || !prev.insertedToSeq){
        signal.insertedToSeq = true;
        std::vector<PointState> vect;
        vect.push_back(signal);
        generatedSequences_.push_back(vect);
        return;
    }

    if (prev.insertedToSeq){
        for( auto & sequence : generatedSequences_){
            if(equalPoints(prev, sequence.end()[-1])){
                signal.insertedToSeq = true;
                sequence.push_back(signal);
                if(sequence.size() > originalSequences_[0].size()){
                    sequence.erase(sequence.begin());
                }
                return;
            }
        }
    }else{
        ROS_ERROR("This is not possible");
    }
}


bool alternativeHT::equalPoints(PointState p1, PointState p2){

    if (p1.point.x == p2.point.x && p1.point.y == p2.point.y && p1.ledState == p2.ledState && p1.insertTime.toSec() == p2.insertTime.toSec()) {
        return true;
    }
    return false; 

}

void alternativeHT::checkIfThreeConsecutiveZeros(){
    
    if (first_call_){
        return;
    }
    std::scoped_lock lock(mutex_buffer);
    std::vector<PointState> & ptsCurr= buffer.end()[-1];
    std::vector<PointState> & ptsSec = buffer.end()[-2];
    std::vector<PointState> & ptsThird = buffer.end()[-3];

    for (int i = 0; i < (int)ptsCurr.size(); i++){
        bool insertedCurr = (ptsCurr[i].insertedToSeq) ? true : false;
        bool boolCurr = ptsCurr[i].ledState; 
        for(int k = 0; k < (int)ptsSec.size(); k++){
            bool insertedSec = (ptsSec[i].insertedToSeq) ? true : false;
            bool boolSec = ptsSec[k].ledState;
            for (int j =0; j<(int)ptsThird.size(); j++){
                bool insertedThird = (ptsThird[i].insertedToSeq) ? true : false;
                bool boolThird = ptsThird[j].ledState; 
                if (insertedCurr && insertedSec && ptsThird[i].insertedToSeq){
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

void alternativeHT::cleanPotentialBuffer(){
    double timeThreeFrames = (1/60.0) * 4; 

    std::scoped_lock lock(mutex_generatedSequences_);

    for (auto it = generatedSequences_.begin(); it != generatedSequences_.end(); ++it){ 
        if (it->empty()){
            continue;
        } 
        std::vector<bool> b;
        // std::cout << "The seq: ";
        // for (auto k : *it){
        //     int i = -1;
        //     if(k.ledState) i = 1; 
        //     else i = 0;
        //     std::cout << k.point.x << "," << k.point.y << "," << i << " - ";
        // }
    //    std::cout << "\n"<< std::endl;


        // printVectorIfNotEmpty(b, "seq");

        bool first = true;
        bool second = true;
        bool third = true;
        if(it->size() >= 3){
            first   = it->end()[-1].ledState;
            second  = it->end()[-2].ledState;
            third   =  it->end()[-3].ledState;
        }
        // std::cout << "The size " << it->size() << " sssss " << (*it).size() << std::endl;

        // if ((*it).size() > originalSequences_[0].size()){
        //     it->erase(it->begin());
        // }

        double insertTime = it->end()[-1].insertTime.toSec(); 
        double timeDiffLastInsert = std::abs(insertTime - ros::Time::now().toSec());
        if (timeDiffLastInsert > timeThreeFrames || (first == false && second == false && third == false) ){ //&& i != generatedSequences_.end()){
            it = generatedSequences_.erase(it);
            continue;
        }

    }
}

std::vector<std::pair<PointState, int>> alternativeHT::getResults(){

    std::scoped_lock lock(mutex_generatedSequences_);
    std::vector<std::pair<PointState, int>> retrievedSignals;
    // std::cout << "++++++++++++++++++++++++\n"; 
    // std::cout << "The list size " << generatedSequences_.size() << "\n";

    for (auto points : generatedSequences_){
        std::vector<bool> ledStates;
        if(points.size() != originalSequences_[0].size()){
            // std::cout << "Not long enough\n"; 
        }
        for (auto point : points){
            ledStates.push_back(point.ledState);
        }
        // double diff =  std::abs(points.end()[-1].insertTime.toSec() - ros::Time::now().toSec()); 
        // std::cout << "time diff " <<  diff << std::endl;
        // printVectorIfNotEmpty(ledStates, "seq");
        int id = findMatch(ledStates);
        std::cout << "The id " << id << std::endl;
        retrievedSignals.push_back(std::make_pair(points.end()[-1], id));
    }

    return retrievedSignals;
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

alternativeHT::~alternativeHT() {
    // delete generatedSequences_;
}



/*****************************************************************************/
// stuff for moving average 
// void uvdar::alternativeHT::checkBoundingBoxIntersection(PointState & point){

//     std::vector<PointState> & ptsCurr   = buffer_.end()[-1];
//     std::vector<PointState> & ptsSec    = buffer_.end()[-2];
//     std::vector<PointState> & ptsThird  = buffer_.end()[-3];
    
//     std::vector<PointState> boxMatchPoints;
//     for (auto & p : ptsSec){
//         if(p.bbLeftUp.x > point.bbRightDown.x || point.bbLeftUp.x > p.bbRightDown.x || 
//         p.bbRightDown.y > point.bbLeftUp.y || point.bbRightDown.y > p.bbLeftUp.y){
//         } else{
//             ROS_INFO("HIT");
//             boxMatchPoints.push_back(p);
//         }
//     }
//     if ((int)boxMatchPoints.size() == 1){

//     } else if ((int)boxMatchPoints.size() > 1){
//         PointState closest  = findClosest(boxMatchPoints, point);
//     } else {
//         ROS_INFO("NO MATCH FOUND - start new sequence!");
//     }

// }

// PointState uvdar::alternativeHT::findClosest(const std::vector<PointState> boxMatchPoints, const PointState point){
//     cv::Point2d min;
//     PointState selected; 
//     min.x = FRAME_SIZE_X;
//     min.y = FRAME_SIZE_Y; 
//     for (const auto p : boxMatchPoints){
//         cv::Point2d diff = computeXYDiff(p.point, point.point);
//         if(diff.x < min.x && diff.y < min.y){
//             selected = p;
//             min.x = diff.x; 
//             min.y = diff.y;
//         }
//     }
//     return selected;
// }
