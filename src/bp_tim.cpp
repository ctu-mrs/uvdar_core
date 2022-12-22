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
    if (initialized_)
        ROS_INFO("[UVDAR_BP_Tim]: Nodelet sucessfully initialized");
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
void uvdar::UVDAR_BP_Tim::processPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptsMsg, const size_t &img_index)
{
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

    processBuffer(small_buffer_[img_index]);

    
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
    }
}

// TODO - CLEAN THIS MESS!
void uvdar::UVDAR_BP_Tim::processBuffer(std::vector<vectPoint3D> & ptsBufferImg)
{

    // std::cout << "P " << small_buffer_[img_index][buffer_cnt_].size() <<std::endl;
    // std::cout << "Psize " << ptsBufferImg[buffer_cnt_].size() <<std::endl;

    // vectPoint3D & currFrame     = ptsBufferImg[buffer_cnt_];
    // std::cout << "Size CURRFRAME" << currFrame.size() << std::endl;
    // vectPoint3D & previousFrame = ptsBufferImg[buffer_cnt_]; // default initializing - will be overridden
    if (first_call_) {
        std::cout << "First call " << std::endl;
        if ( buffer_cnt_ != 0 ) {
            checkInsertVP(ptsBufferImg[buffer_cnt_], ptsBufferImg[buffer_cnt_ - 1]);
        }
        if ( _debug_ ) {
            ROS_INFO("[UVDAR_BP_Tim]: Buffer not filled with enough data.");
        }
        return;
    }

    // not really nice programmed!    
    if (buffer_cnt_ == 0 ){
        if (!checkInsertVP(ptsBufferImg[buffer_cnt_], ptsBufferImg[ _buffer_size_ - 1 ] )){
            findClosestAndLEDState(ptsBufferImg[buffer_cnt_], ptsBufferImg[ _buffer_size_ - 1 ]);
        }
    } else if ( buffer_cnt_ <= _buffer_size_){
        if (!checkInsertVP(ptsBufferImg[ buffer_cnt_ ] , ptsBufferImg[ buffer_cnt_ - 1 ]))
            findClosestAndLEDState(ptsBufferImg[buffer_cnt_], ptsBufferImg[ buffer_cnt_ - 1 ]);
    } else {
        ROS_WARN("[UVDAR_BP_Tim]: Previous Frame and Current Frame pointing to the frame. Algorithm will not work!");
    }

    // go over the "newest" messages in the buffer
    // for (unsigned int i = 0; i < buffer_cnt_; i++)  
    // {
    //     vectPoint3D & olderFrame    = ptsBufferImg[i]; 
    // }

    // go over the "older" messages in the buffer
    // if (!first_call_) { 
    //     if (buffer_cnt_ == 0){
    //     // compare the last element of the buffer with the current element
    //     findClosestAndLEDState(currFrame, ptsBufferImg[_buffer_size_ - 1]);
    //     }
    // }
}


void uvdar::UVDAR_BP_Tim::findClosestAndLEDState(vectPoint3D & ptsCurrentImg, vectPoint3D & ptsPrevImg)
{   

    // std::cout << "Size start" << ptsCurrentImg.size() << " " << ptsOlderImg.size() << std::endl;

    bool nearestNeighbor = true; // bool for predicting the LED state. Assumption: When in both images Points are existent. Some nearest neighbors will be found
    consecutiveFramesZero_ = false;
    // std::cout << "new in find" << ptsNewerImg.size() << std::endl;
    // std::cout << "old in find" << ptsOlderImg.size() << std::endl;

    // if the current frames are both empty return
    if ((ptsCurrentImg.size() == 0) && (ptsPrevImg.size() == 0))
    {
        consecutiveFramesZero_ = true; // if two consecutive frames are empty
        return;
    }
 
    for (auto & pointOlderImg : ptsPrevImg)
    {
        // pointOlderImg.value = 1; // set LED state to "on" because point exists
        for (auto & pointNewerImg : ptsCurrentImg)
        {

            double pixelShift_x = std::abs(pointNewerImg.x - pointOlderImg.x);
            double pixelShift_y = std::abs(pointNewerImg.y - pointOlderImg.y);

            std::cout << "Pixel shift x " << pixelShift_x << " PixelShift y " << pixelShift_y <<  std::endl;

            if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_)
            {
                ROS_WARN("PIXEL SHIFT IS HIGH");
                nearestNeighbor = false;
                // if (_debug_){
                // std::cout << "Nearest Neighbor false!" << std::endl;
                // ROS_INFO("[UVDAR_BP_Tim]: Pixel shift is too big.");
                // }
            }
            else
            {   
                nearestNeighbor = true;
                pointNewerImg.value = 1; // match found - LED is "on"
            }
        }

        if (!nearestNeighbor) { // no Match found! 
        std::cout << "no nearest neighbor" << std::endl;
            // insert coordinate values of the current point into dummy point and set LED state to zero
            insertEmptyPoint(ptsCurrentImg, pointOlderImg);
        }
    }

    for (const auto k : ptsPrevImg ) {
        std::cout << " value old " << k.value << std::endl;
    }
    for (const auto k : ptsCurrentImg ) {
        std::cout << " value new " << k.value << std::endl;
    }
}

// void uvdar::UVDAR_BP_Tim::processSunPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptMsg, const size_t & img_index) {
// }

// void uvdar::UVDAR_BP_Tim::getResults(ros::NodeHandle & private_nh_){
// for (size_t i = 0; i < _points_seen_topics.size(); ++i) {

//     timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(_proces_rate)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this, _1, i), false, true));
// }

// }

/**
 * @brief if one of the frames is empty, virtual point is inserted into the empty image
 * 
 * @param ptsCurrentImg 
 * @param ptsOlderImg 
 * @return true 
 * @return false 
 */
bool uvdar::UVDAR_BP_Tim::checkInsertVP(vectPoint3D &ptsCurrentImg, vectPoint3D &ptsOlderImg){
    if (ptsCurrentImg.size() == 0 && ptsOlderImg.size() != 0){
        for ( auto & p : ptsOlderImg)
        {
            insertEmptyPoint(ptsCurrentImg, p);
            // p.value = 1;
        }
        return true; 
    }

    if (ptsOlderImg.size() == 0 && ptsCurrentImg.size() != 0)
    {        
        for ( auto & p : ptsCurrentImg)
        {  
            insertEmptyPoint(ptsOlderImg, p);
            // p.value = 1;
        }     
        return true; 
    }
    return false;
}

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
            processPoint(pointsMessage, image_index);
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
 * @brief insert empty point into point Vector
 *
 * @param pointVector
 * @param point
 */
void uvdar::UVDAR_BP_Tim::insertEmptyPoint(vectPoint3D &pointVector, const mrs_msgs::Point2DWithFloat point)
{   
    std::cout << "Insert empty point" << std::endl;
    mrs_msgs::Point2DWithFloat p;
    p = point;
    p.value = 0; // equals LED "off" state
    pointVector.push_back(p);
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
void uvdar::UVDAR_BP_Tim::loadParams(const bool &printParams, ros::NodeHandle &private_nh_)
{

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
    if (_buffer_size_ > MAX_BUFFER_SIZE)
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The wanted buffer size: " << _buffer_size_ << "is bigger than the maximum buffer size. This might cause tracking and blink extraction failure");
    }
    if (_buffer_size_ < MIN_BUFFER_SIZE)
    {
        ROS_ERROR_STREAM("[UVDAR_BP_Tim]: The wanted buffer size: " << _buffer_size_ << "is smaller than the maximum buffer size. The minimum size for a working system is: " << MIN_BUFFER_SIZE);
    }

    param_loader.loadParam("sequence_file", _sequence_file, std::string());

    private_nh_.param("blinkers_seen_topics", _blinkers_seen_topics, _blinkers_seen_topics);
    private_nh_.param("estimated_framerate_topics", _estimated_framerate_topics, _estimated_framerate_topics);

    private_nh_.param("use_camera_for_visualization", _use_camera_for_visualization_, bool(true));
}

/**
 * @brief initializes the buffer for each camera topic
 *
 * @param buffer
 */
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