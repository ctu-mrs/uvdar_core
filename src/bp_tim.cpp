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
    vectPoint3D p(std::begin(ptsMsg->points), std::end(ptsMsg->points));
    std::cout << "============================================" << std::endl;
    std::cout << "B size" << ptsMsg->points.size() <<std::endl;
    // set the default LED state to "on" for any existent point
    for (auto & k : p ) {
        k.value = 1;  
    }
    small_buffer_[img_index].at(buffer_cnt_) = p;

    // std::cout << "Img ind" << img_index <<std::endl;
    // processBuffer(small_buffer_[img_index]);
    buffer_cnt_++;

    // resets buffer -> override of oldest message
    if (buffer_cnt_ == _buffer_size_)
    {
        // delete - preliminary
        buffer_cnt_ --;
        for (int i = 0; i < buffer_cnt_; i++)
        {   
            // std::cout << "Buffer size iOne " << small_buffer_[img_index][i + 1].size() << " size i " << small_buffer_[img_index][i].size()<< std::endl;
            vectPoint3D & currFrame  = small_buffer_[img_index][i + 1];
            vectPoint3D & olderFrame = small_buffer_[img_index][i];
            std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
            // checkFrameSizes();
            if (! checkInsertVP(currFrame, olderFrame) ){
                findClosestAndLEDState(currFrame, olderFrame);
            }
            std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;

            if (consecutiveFramesZero_){
            potentialSequences.push_back(currFrame);
            std::cout << "Two consecutive frames" << std::endl;
            }

            // potentialSequences.push_back(small_buffer_[img_index][i]);
            // if (potentialSequences.size() >= 20 ){
            //     std::cout << "------------------" << std::endl;
            //     for ( const auto k : potentialSequences ) {
            //         for (const auto r : k ) {
            //             std::cout << r.value << ", ";
            //         }
            //     }
            //     std::cout << "------------------" << std::endl;

            //     potentialSequences.clear();
            // }

        }
        // std::cout  << " size buffer  " << small_buffer_[img_index].size() << std::endl;
        for (int i = 0; i <  small_buffer_[img_index].size(); i++){
            // std::cout  << " size buffer Points  " << small_buffer_[img_index][i].size() << std::endl;
            for (int r = 0; r < small_buffer_[img_index][i].size(); r++)
                potentialSequences.push_back(small_buffer_[img_index][i]);
                // std::cout << "Value " << small_buffer_[img_index][i].at(r).value << std::endl;
        }

        // checkLEDValidity(small_buffer_[img_index]);
        //----------------

        buffer_cnt_ = 0;
        first_call_ = false;
    }

    if (potentialSequences.size() >= 20 ) {
        std::cout << "==============================================" << std::endl;
        for (const auto k : potentialSequences) {
            for (const auto r : k ){
                std::cout << r.value << ",";
            }
        }
        std::cout << "==============================================" << std::endl;
        potentialSequences.clear();

    }
}

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
        std::cout << "Insert New" <<  ptsCurrentImg.size() << std::endl;

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

        std::cout << "Insert Old" <<  ptsOlderImg.size() << std::endl;
        return true; 
    }
    return false;
}


// TODO - CLEAN THIS MESS!
void uvdar::UVDAR_BP_Tim::processBuffer(std::vector<vectPoint3D> &ptsBufferImg)
{
    // go over the "newest" messages in the buffer
    for (unsigned int i = 0; i < buffer_cnt_; i++)
    {
        findClosestAndLEDState(ptsBufferImg[i + 1], ptsBufferImg[i]);
    }

    // go over the "older" messages in the buffer
    if (!first_call_)
    { 
        // compare the last element of the buffer with the first element, as long as counter is not at the end of the buffer
        if (buffer_cnt_ != (_buffer_size_ - 1))
        {
            findClosestAndLEDState(ptsBufferImg[0], ptsBufferImg[_buffer_size_ - 1]);
        }

        for (int i = buffer_cnt_ + 1; i < (_buffer_size_ - 1); i++)
        {
            findClosestAndLEDState(ptsBufferImg[i + 1], ptsBufferImg[i]);
        }
    }
}

void uvdar::UVDAR_BP_Tim::findClosestAndLEDState(vectPoint3D &ptsCurrentImg, vectPoint3D &ptsOlderImg)
{   
    std::cout << "Size start" << ptsCurrentImg.size() << " " << ptsOlderImg.size() << std::endl;

    bool nearestNeighbor = true; // bool for predicting the LED state.
    // Assumption: When in both images Points are existent. Some nearest neighbors will be found
    consecutiveFramesZero_ = false;
    // std::cout << "new in find" << ptsNewerImg.size() << std::endl;
    // std::cout << "old in find" << ptsOlderImg.size() << std::endl;

   
    // if the current frames are both empty return
    if ((ptsCurrentImg.size() == 0) && (ptsOlderImg.size() == 0))
    {
        consecutiveFramesZero_ = true; // if two consecutive frames are empty
        return;
    }
 
    // when point exists in current image, set LED-state by default to "on"
    // PROBABLY NOT NEEDED
    // for (auto &p : ptsNewerImg)
    // {
    //     p.value = 1;
    // }

    for (auto &pointOlderImg : ptsOlderImg)
    {
        // pointOlderImg.value = 1; // set LED state to "on" because point exists
        for (auto &pointNewerImg : ptsCurrentImg)
        {

            double pixelShift_x = std::abs(pointNewerImg.x - pointOlderImg.x);
            double pixelShift_y = std::abs(pointNewerImg.y - pointOlderImg.y);

            // std::cout << "Pixel shift x " << pixelShift_x << " PixelShift y " << pixelShift_y <<  std::endl;

            if (pixelShift_x > max_pixel_shift_x_ || pixelShift_y > max_pixel_shift_y_)
            {
                nearestNeighbor = false;
                // if (_debug_){
                // std::cout << "Nearest Neighbor false!" << std::endl;
                // ROS_INFO("[UVDAR_BP_Tim]: Pixel shift is too big.");
                // }
            }
            else
            {
                // std::cout << "Nearest Neighbor true!" << std::endl;
                nearestNeighbor = true;
                pointNewerImg.value = 1; // match found - LED is "on"
            }
        }

        // std::cout << "TIME" << ros::Time::now().toSec() << std::endl;
        if (!nearestNeighbor)
        { // no Match found!
            // insert coordinate values of the current point into dummy point and set LED state to zero
            insertEmptyPoint(ptsCurrentImg, pointOlderImg);
        }
    }

    for (const auto k : ptsOlderImg ) {
        std::cout << " value old " << k.value << std::endl;
    }
    for (const auto k : ptsCurrentImg ) {
        std::cout << " value new " << k.value << std::endl;
    }

    // std::cout << "Size ende" << ptsNewerImg.size() << " " << ptsOlderImg.size() << std::endl;



    // here the LED States needs to be checked: e.g. 3x Off/On not possible!
}

// void uvdar::UVDAR_BP_Tim::processSunPoint(const mrs_msgs::ImagePointsWithFloatStampedConstPtr &ptMsg, const size_t & img_index) {
// }

// void uvdar::UVDAR_BP_Tim::getResults(ros::NodeHandle & private_nh_){
// for (size_t i = 0; i < _points_seen_topics.size(); ++i) {

//     timer_process_.push_back(private_nh_.createTimer(ros::Duration(1.0/(double)(_proces_rate)), boost::bind(&UVDAR_BP_Tim::ProcessThread, this, _1, i), false, true));
// }

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
    for (const auto i : _points_seen_topics)
    {
        std::vector<vectPoint3D> b;
        for (int k = 0; k < _buffer_size_; k++)
        {
            vectPoint3D point;
            b.push_back(point);
        }
        small_buffer_.push_back(b);
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uvdar::UVDAR_BP_Tim, nodelet::Nodelet);