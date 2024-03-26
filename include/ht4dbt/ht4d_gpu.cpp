#include "ht4d_gpu.h"

using namespace uvdar;

HT4DBlinkerTrackerGPU::HT4DBlinkerTrackerGPU (
    int i_mem_steps,
    int i_pitch_steps,
    int i_yaw_steps,
    int i_max_pixel_shift,
    cv::Size i_im_res,
    int i_allowed_BER_per_seq,
    int i_nullify_radius,
    int i_reasonable_radius,
    double i_framerate) : HT4DBlinkerTracker(i_mem_steps, i_pitch_steps, i_yaw_steps, i_max_pixel_shift, i_im_res, i_allowed_BER_per_seq, i_nullify_radius,
    i_reasonable_radius, i_framerate) {
  std::cout << "Initiating HT4DBlinkerTrackerGPU..." << std::endl;

  initialized_ = false;
  local_size_x = 16;
  local_size_y = 16;
  pitch_overlap_frac = 4;
  yaw_overlap_frac = 4;
  run_idx = 0;
  max_clusters = 150;

  max_t_points_total = mem_steps_ * 30;

  // prepare pitch and yaw values
  float_pitch_vals = (float*) calloc(pitch_steps_, sizeof(float));
  float_yaw_vals = (float*) calloc(yaw_steps_, sizeof(float));

  for (int i_p = 0; i_p < pitch_steps_; i_p++) {
      float_pitch_vals[i_p] = (float) pitch_vals_.at(i_p);
  }
  for (int i_y = 0; i_y < yaw_steps_; i_y++) {
      float_yaw_vals[i_y] = (float) yaw_vals_.at(i_y);
  }

  std::cout << "...finished." << std::endl;
  return;
}

HT4DBlinkerTrackerGPU::~HT4DBlinkerTrackerGPU() {
  deinit();
  free(float_pitch_vals);
  free(float_yaw_vals);
}

void HT4DBlinkerTrackerGPU::init() {
  if (!initialized_) {
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
    if (asprintf(&formatted_src, ht3dlines_shader_src, local_size_x, local_size_y, pitch_steps_, yaw_steps_, mem_steps_, pitch_overlap_frac, yaw_overlap_frac) < 0)
    {
        fprintf(stderr, "Failed to format shader source!\r\n");
        compute_lib_error_str(code, err_str, &err_str_len);
        return;
    }
    compute_prog = COMPUTE_LIB_PROGRAM_NEW(&compute_inst, formatted_src);
    if (compute_lib_program_init(&compute_prog)) {
        fprintf(stderr, "Failed to create program!\r\n");
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    // init image2d objects
    hough_space_flattened = COMPUTE_LIB_IMAGE2D_NEW("hough_space_flattened", GL_TEXTURE0, im_res_.width, im_res_.height, GL_R32UI, GL_READ_WRITE, GL_CLAMP_TO_EDGE, GL_LINEAR, GL_RED_INTEGER, GL_UNSIGNED_INT);
    if (compute_lib_image2d_init(&compute_prog, &hough_space_flattened, GL_COLOR_ATTACHMENT0)) {
        fprintf(stderr, "Failed to create image2d '%s'!\r\n", hough_space_flattened.uniform_name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    // init SSBOs
    pitch_vals_ssbo = COMPUTE_LIB_SSBO_NEW("pitch_vals_buffer", GL_FLOAT, GL_STATIC_READ);
    if (compute_lib_ssbo_init(&compute_prog, &pitch_vals_ssbo, float_pitch_vals, pitch_steps_)) {
        fprintf(stderr, "Failed to create shader storage buffer '%s'!\r\n", pitch_vals_ssbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }
    yaw_vals_ssbo = COMPUTE_LIB_SSBO_NEW("yaw_vals_buffer", GL_FLOAT, GL_STATIC_READ);
    if (compute_lib_ssbo_init(&compute_prog, &yaw_vals_ssbo, float_yaw_vals, yaw_steps_)) {
        fprintf(stderr, "Failed to create shader storage buffer '%s'!\r\n", yaw_vals_ssbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }
    t_points_ssbo = COMPUTE_LIB_SSBO_NEW("t_points_buffer", GL_INT, GL_STATIC_READ);
    if (compute_lib_ssbo_init(&compute_prog, &t_points_ssbo, NULL, 0)) {
        fprintf(stderr, "Failed to create shader storage buffer '%s'!\r\n", t_points_ssbo.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    // init uniform objects
    tex_loc_uniform = COMPUTE_LIB_UNIFORM_NEW("tex_loc");
    if (compute_lib_uniform_init(&compute_prog, &tex_loc_uniform)) {
        fprintf(stderr, "Failed to create uniform '%s'!\r\n", tex_loc_uniform.name);
        compute_lib_error_queue_flush(&compute_inst, stderr);
        return;
    }

    initialized_ = true;
  }
}

void HT4DBlinkerTrackerGPU::deinit() {
  if (initialized_) {
    // destroy SSBO objects
    compute_lib_ssbo_destroy(&compute_prog, &pitch_vals_ssbo);
    compute_lib_ssbo_destroy(&compute_prog, &yaw_vals_ssbo);
    compute_lib_ssbo_destroy(&compute_prog, &t_points_ssbo);

    // destroy image2d objects
    compute_lib_image2d_destroy(&compute_prog, &hough_space_flattened);

    // destroy compute program
    compute_lib_program_destroy(&compute_prog, true);

    // destroy compute lib instance
    compute_lib_deinit(&compute_inst);

    initialized_ = false;
  }
}

void HT4DBlinkerTrackerGPU::updateResolution(cv::Size i_size){
  updateInterfaceResolution(i_size);
  init();

  if (initialized_) {
    compute_lib_image2d_destroy(&compute_prog, &hough_space_flattened);
    hough_space_flattened = COMPUTE_LIB_IMAGE2D_NEW("hough_space_flattened", GL_TEXTURE0, i_size.width, i_size.height, GL_R32UI, GL_READ_WRITE, GL_CLAMP_TO_EDGE, GL_LINEAR, GL_RED_INTEGER, GL_UNSIGNED_INT);
    if (compute_lib_image2d_init(&compute_prog, &hough_space_flattened, GL_COLOR_ATTACHMENT0)) {
        compute_lib_error_queue_flush(&compute_inst, stderr);
    }
  }
}

std::vector< std::pair<cv::Point2d,int> > HT4DBlinkerTrackerGPU::getResults() {
  if (!getResultsStart()) {
    return std::vector<std::pair<cv::Point2d,int>>();
  }

  // create clusters from accumulator
  uint32_t i_cl;
  t_points_cluster_t* clusters = (t_points_cluster_t*) calloc(max_clusters, sizeof(t_points_cluster_t));
  for (i_cl = 0; i_cl < max_clusters; i_cl++) {
    clusters[i_cl].t_points_array = (ivec4_t*) calloc(max_t_points_total, sizeof(ivec4_t));
  }
  uint32_t clusters_len = 0;
  prepareClusters(clusters, &clusters_len);

  if (debug_)
    std::cout << "[HT4DBlinkerTrackerGPU] Created " << clusters_len << " t-point clusters." << std::endl;

  // pass clusters to the GPU, dispatch shader
  int32_t span_x, span_y;
  int32_t tex_loc[3];

  // reset whole hough space to zeros - not needed thanks to run_idx checks
  //compute_lib_image2d_reset(&compute_prog, &hough_space_flattened, &zero);
  tex_loc[2] = ((run_idx++) % 2) + 1;

  rgba_t* hough_space = (rgba_t*) compute_lib_image2d_alloc(&hough_space_flattened);

  // iterate over t-point clusters
  for (i_cl = 0; i_cl < clusters_len; i_cl++) {
      span_x = clusters[i_cl].max_x - clusters[i_cl].min_x;
      span_y = clusters[i_cl].max_y - clusters[i_cl].min_y;

      // write current cluster to the GPU SSBO
      compute_lib_ssbo_write(&compute_prog, &t_points_ssbo, clusters[i_cl].t_points_array, 4 * clusters[i_cl].num_pts);

      // write hough space image2d start coordinates to the GPU
      tex_loc[0] = clusters[i_cl].min_x;
      tex_loc[1] = clusters[i_cl].min_y;
      compute_lib_uniform_write(&compute_prog, &tex_loc_uniform, tex_loc);

      // dispatch the compute shader
      compute_lib_program_dispatch(&compute_prog, span_x / local_size_x, span_y / local_size_y, 1);
  }

  // read the final flattened hough space from GPU
  //compute_lib_image2d_read(&compute_prog, &hough_space_flattened, hough_space);
  for (i_cl = 0; i_cl < clusters_len; i_cl++) {
      compute_lib_image2d_read_patch(&compute_prog, &hough_space_flattened, hough_space, clusters[i_cl].min_x, clusters[i_cl].max_x, clusters[i_cl].min_y, clusters[i_cl].max_y, i_cl == 0);
  }
  
  // fill matrices using RGBA channels
  union {
      rgba_t rgba;
      struct {
          uint8_t max_val;
          uint8_t i_y;
          uint8_t i_p;
          uint8_t touched;
      };
  } curr;
  int x, y;
  for (y = 0; y < im_res_.height; y++) {
    for (x = 0; x < im_res_.width; x++) {
      curr.rgba = hough_space[index2d(x, y)];
      hough_space_maxima_[index2d(x, y)]      = curr.max_val; //assign the maximum value to this 2D matrix 
      index_matrix_.at< unsigned char >(y, x) = indexYP(curr.i_p, curr.i_y); //assign the index of the maximum to this 2D matrix 
    }
  }

  // free resources
  free(hough_space);
  for (i_cl = 0; i_cl < max_clusters; i_cl++) {
    free(clusters[i_cl].t_points_array);
  }
  free(clusters);

  return getResultsEnd();
}

extern "C" int compare_t_points_xy1d(const void* a, const void* b) {
    ivec3_t* pt1 = (ivec3_t*) a;
    ivec3_t* pt2 = (ivec3_t*) b;
    int res = pt1->y - pt2->y;
    if (res == 0) res = pt1->x - pt2->x;
    return res;
}

void HT4DBlinkerTrackerGPU::prepareClusters(t_points_cluster_t clusters[], uint32_t* clusters_len)
{
    // first sort t-points into array in ascending (y-x) order
    uint32_t t_points_list_len = 0;
    ivec3_t t_points_list[max_t_points_total];
    uint32_t i_pt, t;
    cv::Point2i pt;

    for (t = 0; t < (uint32_t)(accumulator_local_copy_.size()); t++) {
      for (i_pt = 0; i_pt < (uint32_t)(accumulator_local_copy_[t].size()); i_pt++) {
        pt = accumulator_local_copy_[t][i_pt];
        t_points_list[t_points_list_len] = (ivec3_t) {.x = pt.x, .y = pt.y, .z = (int32_t) t};
        t_points_list_len++;
      }
    }

    qsort(t_points_list, t_points_list_len, sizeof(ivec3_t), compare_t_points_xy1d);

    // grow clusters
    bool cluster_found;
    uint32_t x, y;
    uint32_t i_cl;
    for (i_pt = 0; i_pt < t_points_list_len; i_pt++) {
        x = t_points_list[i_pt].x;
        y = t_points_list[i_pt].y;
        t = t_points_list[i_pt].z;

        cluster_found = false;
        for (i_cl = 0; i_cl < *clusters_len; i_cl++) {
            if (x > clusters[i_cl].min_x && x < clusters[i_cl].max_x && y > clusters[i_cl].min_y && y < clusters[i_cl].max_y && clusters[i_cl].num_pts < max_t_points_total) {
                if (x < clusters[i_cl].min_x) clusters[i_cl].min_x = (int) fmaxf(0, x - 2*max_dist);
                if (x > clusters[i_cl].max_x) clusters[i_cl].max_x = (int) fminf(im_res_.width-1, x + 2*max_dist);
                if (y < clusters[i_cl].min_y) clusters[i_cl].min_y = (int) fmaxf(0, y - 2*max_dist);
                if (y > clusters[i_cl].max_y) clusters[i_cl].max_y = (int) fminf(im_res_.height-1, y + 2*max_dist);
                clusters[i_cl].t_points_array[clusters[i_cl].num_pts].x = x;
                clusters[i_cl].t_points_array[clusters[i_cl].num_pts].y = y;
                clusters[i_cl].t_points_array[clusters[i_cl].num_pts].z = t;
                clusters[i_cl].t_points_array[clusters[i_cl].num_pts].w = 1;
                clusters[i_cl].num_pts++;
                cluster_found = true;
                break;
            }
        }

        if (!cluster_found && *clusters_len < max_clusters) {
            clusters[*clusters_len].min_x = (uint32_t) fmaxf(0, x - 2*max_dist);
            clusters[*clusters_len].max_x = (uint32_t) fminf(im_res_.width-1, x + 2*max_dist);
            clusters[*clusters_len].min_y = (uint32_t) fmaxf(0, y - 2*max_dist);
            clusters[*clusters_len].max_y = (uint32_t) fminf(im_res_.height-1, y + 2*max_dist);
            clusters[*clusters_len].num_pts = 0;
            clusters[*clusters_len].t_points_array[clusters[*clusters_len].num_pts].x = x;
            clusters[*clusters_len].t_points_array[clusters[*clusters_len].num_pts].y = y;
            clusters[*clusters_len].t_points_array[clusters[*clusters_len].num_pts].z = t;
            clusters[*clusters_len].t_points_array[clusters[*clusters_len].num_pts].w = 1;
            clusters[*clusters_len].num_pts++;
            *clusters_len += 1;
        }
    }

    for (i_cl = 0; i_cl < *clusters_len; i_cl++) {
        clusters[i_cl].min_x = (uint32_t) fmaxf(0, local_size_x * floorf(((float) clusters[i_cl].min_x) / local_size_x));
        clusters[i_cl].max_x = (uint32_t) fminf(im_res_.width, local_size_x * ceilf(((float) clusters[i_cl].max_x) / local_size_x));
        clusters[i_cl].min_y = (uint32_t) fmaxf(0, local_size_y * floorf(((float) clusters[i_cl].min_y) / local_size_y));
        clusters[i_cl].max_y = (uint32_t) fminf(im_res_.height, local_size_y * ceilf(((float) clusters[i_cl].max_y) / local_size_y));

        //printf("Cluster #%d: min_x=%d, max_x=%d, min_y=%d, max_y=%d\r\n", i_cl, clusters[i_cl].min_x, clusters[i_cl].max_x, clusters[i_cl].min_y, clusters[i_cl].max_y);

        for (i_pt = 0; i_pt < clusters[i_cl].num_pts; i_pt++) {
            clusters[i_cl].t_points_array[i_pt].x -= clusters[i_cl].min_x;
            clusters[i_cl].t_points_array[i_pt].y -= clusters[i_cl].min_y;
        }
    }
}

