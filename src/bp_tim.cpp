#include <bp_tim.h>

// default constructor
uvdar::UVDAR_BP_Tim::UVDAR_BP_Tim(){};

// destructor
uvdar::UVDAR_BP_Tim::~UVDAR_BP_Tim(){};

void uvdar::UVDAR_BP_Tim::onInit()
{

    ros::NodeHandle &private_nh_ = getMTPrivateNodeHandle();
    NODELET_DEBUG("[UVDAR_BP_Tim]: Initializing Nodelet...");

    const bool printParams = false;

    loadParams(printParams, private_nh_);
    parseSequenceFile(_sequence_file);

    if (!checkVectorSizeMatch(_blinkers_seen_topics, _estimated_framerate_topics, _points_seen_topics))
        return;

    // setup data structure
    initSmallBuffer();
    
    for (size_t i = 0; i < _points_seen_topics.size(); ++i) {
        aht_.push_back(
                std::make_shared<alternativeHT>( _buffer_size_ )
              );
        //   ht4dbt_trackers_.back()->setDebug(_debug_, _visual_debug_);
        //   ht4dbt_trackers_.back()->setSequences(_sequences_);

        //   blink_data_.push_back(BlinkData());

        //   mutex_camera_image_.push_back(std::make_shared<std::mutex>());
        //   camera_image_sizes_.push_back(cv::Size(-1,-1));
    }


    first_call_ = true;
    subscribeToPublishedPoints(private_nh_);

    // for (size_t i = 0; i < _blinkers_seen_topics.size(); ++i) {
    //     pub_blinkers_seen_.push_back(private_nh_.advertise<mrs_msgs::ImagePointsWithFloatStamped>(_blinkers_seen_topics[i], 1));
    //     pub_estimated_framerate_.push_back(private_nh_.advertise<std_msgs::Float32>(_estimated_framerate_topics[i], 1));
    // }

    // current_visualization_done_ = false;
    // timer_visualization_ = private_nh_.createTimer(ros::Rate(_visualization_rate_), &UVDAR_BP_Tim::VisualizationThread, this, false);

    // getResults(private_nh_);

    initialized_ = true;
}

/**
 * @brief called with interval of frame rate of camera
 * - ptsMsg is empty if no points seen
 * - the Buffer vector is filled with recurring fashion
 *      -> if counter is at the end of the buffer, the counter is reset to zero
 *
 * @param ptsMsg
 * @param img_index
 * @return * void
 */
void uvdar::UVDAR_BP_Tim::insertPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptsMsg, const size_t &img_index)
{
    if (!initialized_) return;
    vectPoint3D points(std::begin(ptsMsg->points), std::end(ptsMsg->points));
    std::cout << "============================================" << std::endl;
    std::cout << "B size" << ptsMsg->points.size() << " Buffer cnt " << buffer_cnt_ <<std::endl;
    
    // vectPoint3D & currFrame  = small_buffer_[img_index][buffer_cnt_];

    // set the default LED state to "on" for any existent point
    for (auto & point : points ) {
        point.value = 1;  
    }

    small_buffer_[img_index][buffer_cnt_] = points;

    for (const auto l : small_buffer_[img_index][buffer_cnt_] ){
        std::cout << "V = " << l.value << ", " << std::endl; 
    }

    aht_[img_index]->processBuffer(small_buffer_[img_index], buffer_cnt_);

    
    potentialSequences.push_back(small_buffer_[img_index][buffer_cnt_]);  
    if ( potentialSequences.size() >= 20 ){
        std::cout << "The sequence is : " << std::endl;
        for (const auto l : potentialSequences ){
            for (const auto r : l) {
                std::cout << r.value << ", "; 
            }
        }
        potentialSequences.clear();
    }
        std::cout << std::endl;
    
    // }



    buffer_cnt_++;

    // resets buffer -> start overriding older messages
    if (buffer_cnt_ == _buffer_size_) {
        buffer_cnt_ = 0;
        first_call_ = false; 


        // set the first_call_ bool once
        /*
        Code within lambda function is executed only once, when the static variable is initialized to the return value of lambda function. It should be thread-safe as long as your compiler support thread-safe static initialization. */
        static bool once = [i=img_index, firstCall = first_call_, this](){
            aht_[i]->setFirstCallBool(firstCall);
            return true;
        }();

    }
}   

   
// void uvdar::UVDAR_BP_Tim::processSunPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptMsg, const size_t & img_index) {
// }

/**
 * @brief Callback for points published by each camera
 *
 * @param private_nh_
 */
void uvdar::UVDAR_BP_Tim::subscribeToPublishedPoints(ros::NodeHandle &private_nh_)
{

    for (size_t i = 0; i < _points_seen_topics.size(); ++i)
    {
        // Subscribe to corresponding topics
        points_seen_callback_t callback = [image_index = i, this](const mrs_msgs::ImagePointsWithFloatStampedConstPtr &pointsMessage)
        {
            insertPoint(pointsMessage, image_index);
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

/**
 * @brief Checks if the _blinkers_seen_topics and _estimated_framerate_topics equals the size of _points_seen_topics
 *
 * @param _blinkers_seen_topics
 * @param _points_seen_topics
 * @param _estimated_framerate_topics
 */
bool uvdar::UVDAR_BP_Tim::checkVectorSizeMatch(const std::vector<std::string> &_blinkers_seen_topics, const std::vector<std::string> &_estimated_framerate_topics, const std::vector<std::string> &_points_seen_topics)
{

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


/**
 * @brief Loads the file with lines describing useful blinking singals
 *
 * @param sequence_file The input file name
 *
 * @return Success status
 */
bool uvdar::UVDAR_BP_Tim::parseSequenceFile(const std::string &sequence_file)
{
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
        _sequences_ = sequences;
    }
    else
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: Failed to load sequence file " << sequence_file << "! Returning.");
        ifs.close();
        return false;
    }
    return true;
}

/**
 * @brief calls ParamLoad and loads parameters from launch file
 *
 * @param printParams
 * @param private_nh_
 */
void uvdar::UVDAR_BP_Tim::loadParams(const bool &printParams, ros::NodeHandle &private_nh_){

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

// /**
//  * @brief initializes the buffer for each camera topic
//  *
//  * @param buffer
//  */
void uvdar::UVDAR_BP_Tim::initSmallBuffer()
{

    small_buffer_.reserve(_points_seen_topics.size());
    potentialSequences.reserve(_points_seen_topics.size());
    for (const auto i : _points_seen_topics)
    {
        std::vector<vectPoint3D> b;
        for (int k = 0; k < _buffer_size_; k++)
        {
            vectPoint3D point;
            b.push_back(point);
        }
        small_buffer_.push_back(b);
        // potentialSequences.push_back(b);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);