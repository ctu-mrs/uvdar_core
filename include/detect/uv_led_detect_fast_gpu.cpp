#include <dirent.h>
#include <algorithm>
#include <iostream>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "uv_led_detect_fast_gpu.h"

//addressing indices this way is noticeably faster than the "propper" way with .at method - numerous unnecessary checks are skipped. This of course means that we have to do necessary checks ourselves
#define index2d(X, Y) (image_curr_.cols * (Y) + (X))

bool uvdar::UVDARLedDetectFASTGPU::initDelayed(const cv::Mat i_image){
  image_size = i_image.size();
  init();
  return initialized_;
}

uvdar::UVDARLedDetectFASTGPU::UVDARLedDetectFASTGPU(bool i_gui, bool i_debug, int i_threshold, int i_threshold_diff, int i_threshold_sun, std::vector<cv::Mat> i_masks) : UVDARLedDetectFAST(i_gui, i_debug, i_threshold, i_threshold_diff, i_threshold_sun, i_masks) {
  local_size_x = 16;
  local_size_y = 16;
  max_markers_count = 300;
  max_sun_pts_count = 10000;
}

void uvdar::UVDARLedDetectFASTGPU::init() {
  if (initialized_) {
    return;
  }

  int code;
    char err_str[4096];
    int err_str_len = 0;

    // init compute lib instance
    compute_inst = COMPUTE_LIB_INSTANCE_NEW;
    if ((code = compute_lib_init(&compute_inst))) {
        compute_lib_error_str(code, err_str, &err_str_len);
        fprintf(stderr, "%.*s", err_str_len, err_str);
        return;
    }

    // init compute program
    char* formatted_src;
    if (asprintf(&formatted_src, fastlike_shader_src, local_size_x, local_size_y, _threshold_, _threshold_diff_, _threshold_sun_, max_markers_count, max_sun_pts_count) < 0)
    {
        fprintf(stderr, "Failed to format shader source!\r\n");
        compute_lib_error_str(code, err_str, &err_str_len);
        return;
    }
    compute_prog = COMPUTE_LIB_PROGRAM_NEW(&compute_inst, formatted_src);
    if (compute_lib_program_init(&compute_prog)) {
        fprintf(stderr, "Failed to create program!\r\n");
        free(formatted_src);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    //compute_lib_program_print_resources(&compute_prog);

    // init image2d objects
   texture_in = COMPUTE_LIB_IMAGE2D_NEW("image_in", GL_TEXTURE0, image_size.width, image_size.height, GL_R8UI, GL_READ_ONLY, GL_CLAMP_TO_EDGE, GL_LINEAR, GL_RED_INTEGER, GL_UNSIGNED_BYTE);
    if (compute_lib_image2d_init(&compute_prog, &texture_in, 0)) {
        fprintf(stderr, "Failed to create image2d '%s'!\r\n", texture_in.uniform_name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    mask = COMPUTE_LIB_IMAGE2D_NEW("mask", GL_TEXTURE1, image_size.width, image_size.height, GL_R8UI, GL_READ_ONLY, GL_CLAMP_TO_EDGE, GL_LINEAR, GL_RED_INTEGER, GL_UNSIGNED_BYTE);
    if (compute_lib_image2d_init(&compute_prog, &mask, 0)) {
      fprintf(stderr, "Failed to create image2d '%s'!\r\n", mask.uniform_name);
      compute_lib_error_queue_flush(&compute_inst, stderr);
      return;
    }
    
    // init SSBOs
    markers_ssbo = COMPUTE_LIB_SSBO_NEW("markers_buffer", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW);
    if (compute_lib_ssbo_init(&compute_prog, &markers_ssbo, NULL, max_markers_count)) {
        fprintf(stderr, "Failed to create shader storage buffer '%s'!\r\n", markers_ssbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    sun_pts_ssbo = COMPUTE_LIB_SSBO_NEW("sun_pts_buffer", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW);
    if (compute_lib_ssbo_init(&compute_prog, &sun_pts_ssbo, NULL, max_sun_pts_count)) {
        fprintf(stderr, "Failed to create shader storage buffer '%s'!\r\n", sun_pts_ssbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    // init atomic counter buffer objects
    markers_count_acbo = COMPUTE_LIB_ACBO_NEW("markers_count", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW);
    if (compute_lib_acbo_init(&compute_prog, &markers_count_acbo, NULL, 0)) {
        fprintf(stderr, "Failed to create atomic counter '%s'!\r\n", markers_count_acbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }
    sun_pts_count_acbo = COMPUTE_LIB_ACBO_NEW("sun_pts_count", GL_UNSIGNED_INT, GL_DYNAMIC_DRAW);
    if (compute_lib_acbo_init(&compute_prog, &sun_pts_count_acbo, NULL, 0)) {
        fprintf(stderr, "Failed to create atomic counter '%s'!\r\n", sun_pts_count_acbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    // dummy clear of mask
    uint32_t zero = 255;
    compute_lib_image2d_reset(&compute_prog, &mask, &zero);

    initialized_ = true;
}

uvdar::UVDARLedDetectFASTGPU::~UVDARLedDetectFASTGPU() {
  if (initialized_) {
    // destroy SSBO objects
    compute_lib_ssbo_destroy(&compute_prog, &markers_ssbo);
    compute_lib_ssbo_destroy(&compute_prog, &sun_pts_ssbo);

    // destroy atomic counter buffer objects
    compute_lib_acbo_destroy(&compute_prog, &markers_count_acbo);
    compute_lib_acbo_destroy(&compute_prog, &sun_pts_count_acbo);

    // destroy image2d objects
    compute_lib_image2d_destroy(&compute_prog, &texture_in);
    compute_lib_image2d_destroy(&compute_prog, &mask);

    // destroy compute program
    compute_lib_program_destroy(&compute_prog, true);

    // destroy compute lib instance
    compute_lib_deinit(&compute_inst);
  }
}

bool uvdar::UVDARLedDetectFASTGPU::processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id) {
  detected_points = std::vector<cv::Point2i>();
  sun_points = std::vector<cv::Point2i>();
  image_curr_     = i_image;

    /* std::cerr << "[UVDARDetectorFASTGPU]: Getting image..." << std::endl; */

  if (!initialized_) {
    /* image_size = i_image.size(); */
    std::cerr << "[UVDARDetectorFASTGPU]: Not yet initialized, returning..." << std::endl;
    /* init(); */
    return false;
  }

  if (mask_id >= (int)(masks_.size())) {
    std::cerr << "[UVDARDetectorFASTGPU]: Mask index " << mask_id << " is greater than the current number of loaded masks!" << std::endl;
    return false;
  }

  if (mask_id >= 0){
    if (image_curr_.size() != masks_[mask_id].size()) {
      std::cerr << "[UVDARDetectorFASTGPU]: The size of the selected mask does not match the current image!" << std::endl;
      return false;
    }
  }

  if (_gui_) {
    (image_curr_).copyTo(image_view_);
  }

  fast_det_pt_t markers[max_markers_count];
  fast_det_pt_t sun_pts[max_sun_pts_count];
  uint32_t sun_points_cnt_val;
  uint32_t markers_cnt_val;

  // reset atomic counter buffer objects
  compute_lib_acbo_write_uint_val(&compute_prog, &markers_count_acbo, 0);
  compute_lib_acbo_write_uint_val(&compute_prog, &sun_pts_count_acbo, 0);
  
  // write input image data + mask data to GPU
  compute_lib_image2d_write(&compute_prog, &texture_in, image_curr_.data);
  compute_lib_image2d_write(&compute_prog, &mask, (mask_id>=0)?masks_[mask_id].data:nullptr);

  /* std::cerr << "[UVDARDetectorFASTGPU]: Dispatching..." << std::endl; */
  // dispatch compute shader
  if ( compute_lib_program_dispatch(&compute_prog, image_size.width / local_size_x, image_size.height / local_size_y, 1)){
      std::cerr << "[UVDARDetectorFASTGPU]: Failed to dispatch the shader!" << std::endl;
      return false;
  }

  /* std::cerr << "[UVDARDetectorFASTGPU]: Retrieving markers..." << std::endl; */
  // retrieve detected markers
  if ( compute_lib_acbo_read_uint_val(&compute_prog, &markers_count_acbo, &markers_cnt_val)){
    std::cerr << "[UVDARDetectorFASTGPU]: Failed to extract the marker count!" << std::endl;
    return false;
  }
  if (markers_cnt_val > max_markers_count) markers_cnt_val = max_markers_count;
  if (markers_cnt_val > 0)
  {
    if ( compute_lib_ssbo_read(&compute_prog, &markers_ssbo, (void*) markers, markers_cnt_val)){
    std::cerr << "[UVDARDetectorFASTGPU]: Failed to extract the marker SSBO!" << std::endl;
    return false;
    }
  }

  /* std::cerr << "[UVDARDetectorFASTGPU]: Retrieving sun points..." << std::endl; */
  // retrieve detected sun points
  if (compute_lib_acbo_read_uint_val(&compute_prog, &sun_pts_count_acbo, &sun_points_cnt_val)){
    std::cerr << "[UVDARDetectorFASTGPU]: Failed to extract the marker count!" << std::endl;
    return false;
  }
  if (sun_points_cnt_val > max_sun_pts_count) sun_points_cnt_val = max_sun_pts_count;
  if (sun_points_cnt_val > 0)
  {
    if (compute_lib_ssbo_read(&compute_prog, &sun_pts_ssbo, (void*) sun_pts, sun_points_cnt_val)){
      std::cerr << "[UVDARDetectorFASTGPU]: Failed to extract the marker SSBO!" << std::endl;
      return false;
    }
  }

  /* std::cerr << "[UVDARDetectorFASTGPU]: Calculating centoids..." << std::endl; */
  // find centroids of concentrated detected markers
  cpuFindMarkerCentroids(markers, markers_cnt_val, 5, detected_points);

  for (uint32_t i = 0; i < sun_points_cnt_val; i++){
    sun_points.push_back(cv::Point(sun_pts[i].x,sun_pts[i].y));
  }

  /* for (int i = 0; i< sun_points_cnt_val; i++){ */
  /*   std::cout << "Sun pt: " << sun_points[i].x << ":" << sun_points[i].y << std::endl; */
  /* } */
  /* for (int i = 0; i< markers_cnt_val; i++){ */
  /*   std::cout << "Found: " << markers[i].x << ":" << markers[i].y << std::endl; */
  /* } */
  /* for (auto p : detected_points){ */
  /*   std::cout << "Refined: " << p << std::endl; */
  /* } */

  /* std::cerr << "[UVDARDetectorFASTGPU]: Filtering markers based on sun points..." << std::endl; */
  // filter markers using detected sun points
  for (int i = 0; i < (int)(detected_points.size()); i++) { //iterate over the detected marker points
    for (int j = 0; j < (int)(sun_points.size()); j++) { //iterate over the detected sun points
      if (cv::norm(detected_points[i] - (sun_points[j])) < 25) { //if the current detected marker point is close to the sun, it might be merely glare, so we discard it rather than to have numerous false detections here
        detected_points.erase(detected_points.begin() + i);
        i--;
        break;
      }
    }
  }

  return true;
}

extern "C" int compare_fast_det_pt_xy1d(const void* a, const void* b) {
    fast_det_pt_t* pt1 = (fast_det_pt_t*) a;
    fast_det_pt_t* pt2 = (fast_det_pt_t*) b;
    int res = pt1->y - pt2->y;
    if (res == 0) res = pt1->x - pt2->x;
    return res;
}

uint32_t uvdar::UVDARLedDetectFASTGPU::cpuFindMarkerCentroids(fast_det_pt_t* markers, uint32_t init_cnt, uint32_t distance_px, std::vector<cv::Point2i>& detected_points) {
    ivec3_t filtered_markers[init_cnt];
    uint32_t filtered_cnt = 0;
    uint32_t max_dist2 = (distance_px * distance_px);
    uint32_t min_dist2, dist2;
    int32_t closest_marker;
    uint32_t i, j;
    int32_t x2, y2, n;
    fast_det_pt_t pt;

    qsort(markers, init_cnt, sizeof(fast_det_pt_t), compare_fast_det_pt_xy1d);

    for (i = 0; i < init_cnt; i++) {
        pt = markers[i];

        min_dist2 = max_dist2;
        closest_marker = -1;

        for (j = 0; j < filtered_cnt; j++) {
            n = filtered_markers[j].z;
            x2 = filtered_markers[j].x / n;
            y2 = filtered_markers[j].y / n;
            dist2 = (x2 - pt.x)*(x2 - pt.x) + (y2 - pt.y)*(y2 - pt.y);
            if (dist2 < min_dist2) {
                min_dist2 = dist2;
                closest_marker = j;
            }
        }

        if (closest_marker != -1) {
            filtered_markers[closest_marker].x += pt.x;
            filtered_markers[closest_marker].y += pt.y;
            filtered_markers[closest_marker].z += 1;
        } else {
            filtered_markers[filtered_cnt].x = pt.x;
            filtered_markers[filtered_cnt].y = pt.y;
            filtered_markers[filtered_cnt].z = 1;
            filtered_cnt++;
        }
    }

    for (i = 0; i < filtered_cnt; i++) {
        detected_points.push_back(cv::Point(filtered_markers[i].x / filtered_markers[i].z, filtered_markers[i].y / filtered_markers[i].z));
    }
    return filtered_cnt;
}

