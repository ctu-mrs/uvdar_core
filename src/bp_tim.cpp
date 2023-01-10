#include <bp_tim.h>
#include <mutex>
// default constructor
uvdar::UVDAR_BP_Tim::UVDAR_BP_Tim(){};

// destructor
uvdar::UVDAR_BP_Tim::~UVDAR_BP_Tim(){};

void uvdar::UVDAR_BP_Tim::onInit()
{

    private_nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();
    NODELET_DEBUG("[UVDAR_BP_Tim]: Initializing Nodelet...");

    const bool printParams = false;

    loadParams(printParams);

    const bool match = checkCameraTopicSizeMatch();
    if( !match ) return;

    parseSequenceFile(_sequence_file);
    initAlternativeHTDataStructure();

    first_call_ = true;
    subscribeToPublishedPoints();

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
        timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(process_rate)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this, _1, i), false, true));
    }

    initialized_ = true;
}

void uvdar::UVDAR_BP_Tim::loadParams(const bool &printParams) {

    mrs_lib::ParamLoader param_loader(private_nh_, printParams, "UVDAR_BP_Tim");

    param_loader.loadParam("uav_name", _uav_name_, std::string());
    param_loader.loadParam("debug", _debug_, bool(false));
    param_loader.loadParam("visual_debug", _visual_debug_, bool(false));
    if (_visual_debug_)
    {
        ROS_WARN_STREAM("[UVDAR_BP_Tim]: You are using visual debugging. This option is only meant for development. Activating it significantly increases load on the system and the user should do so with care.");
    }

    param_loader.loadParam("gui", _gui_, bool(true));                                      // currently not used!
    param_loader.loadParam("publish_visualization", _publish_visualization_, bool(false)); // currently not used!
    param_loader.loadParam("visualization_rate", _visualization_rate_, float(2.0));        // currently not used!

    param_loader.loadParam("points_seen_topics", _points_seen_topics, _points_seen_topics);

    param_loader.loadParam("enable_manchester", _enable_manchester_, bool(false));
    if (_enable_manchester_)
        ROS_WARN_STREAM("[UVDARBlinkProcessor]: Manchester Decoding is enabled. Make sure Transmitter has same coding enabled!");

    param_loader.loadParam("buffer_size", _buffer_size_, int(3));
    if ( _buffer_size_ > max_buffer_size_ ) {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The wanted buffer size: " << _buffer_size_ << " is bigger than the maximum buffer size. The maximum buffer size is " << max_buffer_size_ << ". The current setting might cause tracking and blink extraction failure");
    }
    if ( _buffer_size_ < min_buffer_size_ ) {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The wanted buffer size: " << _buffer_size_ << " is smaller than the minimum buffer size. The minimum size for a working system is: " << min_buffer_size_);
    }

    param_loader.loadParam("sequence_file", _sequence_file, std::string());

    private_nh_.param("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
    private_nh_.param("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);

    private_nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));
}

bool uvdar::UVDAR_BP_Tim::parseSequenceFile(const std::string &sequence_file) {

    ROS_WARN_STREAM("[UVDAR_BP_Tim]: Add sanitation - sequences must be of equal, non-zero length");
    ROS_INFO_STREAM("[UVDAR_BP_Tim]: Loading sequence from file: [ " + sequence_file + " ]");
    std::ifstream ifs;
    ifs.open(sequence_file);
    std::string word;
    std::string line;
    std::vector<std::vector<bool>> sequences;
    if (ifs.good())
    {
        ROS_INFO("[UVDAR_BP_Tim]: Loaded Sequences: [: ");
        while (getline(ifs, line))
        {
            if (line[0] == '#')
            {
                continue;
            }
            std::string show_string = "";
            std::vector<bool> sequence;
            std::stringstream iss(line);
            std::string token;
            while (std::getline(iss, token, ','))
            {
                if (!_enable_manchester_)
                {
                    sequence.push_back(token == "1");
                }
                else
                {
                    // Manchester Coding - IEEE 802.3 Conversion: 1 = [0,1]; 0 = [1,0]
                    if (token == "1")
                    {
                        sequence.push_back(false);
                        sequence.push_back(true);
                    }
                    else
                    {
                        sequence.push_back(true);
                        sequence.push_back(false);
                    }
                }
            }
            for (const auto boolVal : sequence)
            {
                if (boolVal)
                    show_string += "1,";
                else
                    show_string += "0,";
            }

            sequences.push_back(sequence);
            ROS_INFO_STREAM("[UVDAR_BP_Tim]: [" << show_string << "]");
        }
        ROS_INFO("[UVDAR_BP_Tim]: ]");
        ifs.close();
        sequences_ = sequences;
    }
    else
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Failed to load sequence file " << sequence_file << "! Returning.");
        ifs.close();
        return false;
    }
    return true;
}

bool uvdar::UVDAR_BP_Tim::checkCameraTopicSizeMatch() {

    if (_blinkers_seen_topics.size() != _points_seen_topics.size())
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim] The number of poinsSeenTopics (" << _points_seen_topics.size()
                                                                          << ") is not matching the number of blinkers_seen_topics (" << _blinkers_seen_topics.size() << ")!");
        return false;
    }
    if (_estimated_framerate_topics.size() != _points_seen_topics.size())
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim] The number of poinsSeenTopics (" << _points_seen_topics.size()
                                                                          << ") is not matching the number of blinkers_seen_topics (" << _estimated_framerate_topics.size() << ")!");
        return false;
    }
    return true;
}

void uvdar::UVDAR_BP_Tim::initAlternativeHTDataStructure(){

    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
        aht_.push_back(
                std::make_shared<alternativeHT>(_buffer_size_)
              );
        if (_enable_manchester_) {
            aht_[i]->setManchesterBoolTrue();
        }
        aht_[i]->setSequences(sequences_);
        aht_[i]->setDebugFlags(_debug_, _visual_debug_);
    }
}

void uvdar::UVDAR_BP_Tim::subscribeToPublishedPoints() {

    for (size_t i = 0; i < _points_seen_topics.size(); ++i)
    {
        // Subscribe to corresponding topics
        points_seen_callback_t callback = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &pointsMessage)
        {
            insertPointToAHT(pointsMessage, image_index);
        };
        cals_points_seen_.push_back(callback);
        sub_points_seen_.push_back(private_nh_.subscribe(_points_seen_topics[i], 1, cals_points_seen_[i]));

        // points_seen_callback_t sun_callback = [image_index=i,this] (const mrs_msgs::ImagePointsWithFloatStampedConstPtr& sunPointsMessage) {
        //     // processSunPoint(sunPointsMessage, image_index);
        // };
        cals_sun_points_.push_back(callback);
        sub_sun_points_.push_back(private_nh_.subscribe(_points_seen_topics[i] + "/sun", 1, cals_sun_points_[i]));
    }
}

void uvdar::UVDAR_BP_Tim::insertPointToAHT(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptsMsg, const size_t &img_index) {
    
    if (!initialized_) return;

    vectPoint3D points(std::begin(ptsMsg->points), std::end(ptsMsg->points));
    
    // std::cout << "============================================" << std::endl;
    // std::cout << "B size" << ptsMsg->points.size() << " Buffer cnt " << buffer_cnt_ <<std::endl;

    // set the default LED state to "on" for any existent point
    for (auto & point : points ) {
        point.value = 1;  
    }
    
    std::vector<std::pair<mrs_msgs::Point2DWithFloat, int>> pointsWithIndex;
    for ( size_t i = 0; i < points.size(); i++ ) {
        points[i].value = 1;
        pointsWithIndex.push_back( std::make_pair( points[i], i ) );
    }

    aht_[img_index]->processBuffer( pointsWithIndex, buffer_cnt_ );
    // aht_[img_index]->processBuffer( points, buffer_cnt_ );

    updateBufferAndSetFirstCallBool(img_index);
 
} 

void uvdar::UVDAR_BP_Tim::updateBufferAndSetFirstCallBool(const size_t & img_index) {

    buffer_cnt_++;

    // resets buffer -> start overriding older messages
    if (buffer_cnt_ == _buffer_size_) {
        buffer_cnt_ = 0;
        
        first_call_ = false; 

        // call flag once - MAYBE NOT WORKING WITH MULTIPLE CAMERAS!!!!!!!!
        [[maybe_unused]] static bool once = [i=img_index, firstCall = first_call_, this](){
            aht_[i]->setFirstCallBool(firstCall);
            std::cout << "FIRST BOOL CALL" << std::endl;
            return true;
        }();

    }
}

void uvdar::UVDAR_BP_Tim::ProcessThread([[maybe_unused]] const ros::TimerEvent& te, size_t image_index){


    // int signa_id;
    aht_[image_index]->getResult();


}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);