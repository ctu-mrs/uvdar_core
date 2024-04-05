#ifndef HT4D_GPU_H
#define HT4D_GPU_H

extern "C" {
#include "../compute_lib/compute_lib.h"
}
#include "ht4d.h"

typedef struct {
    uint32_t min_x;
    uint32_t max_x;
    uint32_t min_y;
    uint32_t max_y;
    ivec4_t* t_points_array;
    uint32_t num_pts;
} t_points_cluster_t;

namespace uvdar {

class HT4DBlinkerTrackerGPU : public HT4DBlinkerTracker {
public:

  HT4DBlinkerTrackerGPU(
      int i_mem_steps,
      int i_pitch_steps,
      int i_yaw_steps,
      int i_max_pixel_shift,
      cv::Size i_im_res,
      int i_allowed_BER_per_seq,
      int i_nullify_radius = 8,
      int i_reasonable_radius = 6,
      double i_framerate = 72);

  ~HT4DBlinkerTrackerGPU();

  std::vector< std::pair<cv::Point2d,int> > getResults();

  void updateResolution(cv::Size i_size);

private:
  void init();
  void deinit();

  void prepareClusters(t_points_cluster_t clusters[], uint32_t* clusters_len);

  bool initialized_;
  uint32_t max_t_points_total;
  int max_dist;
  uint32_t max_clusters;
  uint32_t local_size_x;
  uint32_t local_size_y;
  uint32_t pitch_overlap_frac;
  uint32_t yaw_overlap_frac;
  float* float_pitch_vals;
  float* float_yaw_vals;
  uint32_t run_idx;

  compute_lib_instance_t compute_inst;
  compute_lib_program_t compute_prog;
  compute_lib_image2d_t hough_space_flattened;
  compute_lib_ssbo_t pitch_vals_ssbo, yaw_vals_ssbo, t_points_ssbo;
  compute_lib_uniform_t tex_loc_uniform;

  static constexpr char ht3dlines_shader_src[] = R""""(
#version 320 es

#define LOCAL_SIZE_X %d
#define LOCAL_SIZE_Y %d
#define PARAM_PITCH_NUM %d
#define PARAM_YAW_NUM %d
#define NUM_MEM_STEPS %d
#define PITCH_OVERLAP_FRAC %d
#define YAW_OVERLAP_FRAC %d

#define PI 3.1415926535897932384626433832795f
#define YAW_PITCH_DIM_NUM uint(PARAM_PITCH_NUM*PARAM_YAW_NUM)
#define pow2(x) ((x)*(x))

layout (local_size_x = LOCAL_SIZE_X, local_size_y = LOCAL_SIZE_Y, local_size_z = 1) in;

layout(r32ui, binding = 0) uniform highp uimage2D hough_space_flattened;

layout(std430, binding = 2) readonly buffer t_points_buffer { ivec4 t_points[];};
layout(std430, binding = 3) readonly buffer pitch_vals_buffer { float pitch_vals[PARAM_PITCH_NUM];};
layout(std430, binding = 4) readonly buffer yaw_vals_buffer { float yaw_vals[PARAM_YAW_NUM];};

uniform ivec3 tex_loc;

precision highp float;
int dilation_radius;
int max_pixel_shift;
float step_yaw;
float min_pitch;
float step_pitch;
int max_dist;
int max_i_y, max_i_p, max_val = 0;
        
int col_yp[YAW_PITCH_DIM_NUM];

float diff_wrap(float a, float b, float h)
{
    return mod(a - b + h, 2.0f * h) - h;
}

void increment_col_yp(int i_y, int i_p)
{
    int i_yp = i_y * PARAM_PITCH_NUM + i_p;
    col_yp[i_yp]++;

    if (col_yp[i_yp] > max_val) {
        max_val = col_yp[i_yp];
        max_i_y = i_y;
        max_i_p = i_p;
    }
}

void main()
{
    int x = int(gl_GlobalInvocationID.x);
    int y = int(gl_GlobalInvocationID.y);
    dilation_radius = 1;
    max_pixel_shift = 1;
    step_yaw = float((2.0f * PI) / float(PARAM_YAW_NUM));
    min_pitch = atan(1.0f / float(max_pixel_shift));
    step_pitch = float((PI / 2.0f) - min_pitch) / float(PARAM_PITCH_NUM - 1);
    max_dist = max_pixel_shift * NUM_MEM_STEPS - 1;

    int i, i_y, i_p, i_yp;
    
    int marker_x, marker_y, marker_t;
    float dist, pitch_angle, yaw_angle;
    int yaw_overlap = 0, pitch_overlap = 0;
    
    max_i_y = 0;
    max_i_p = 0;
    max_val = 0;
    
    // clear the column for permuted yaw-pitch indices
    for (i_yp = 0; i_yp < int(YAW_PITCH_DIM_NUM); i_yp++) {
        col_yp[i_yp] = 0;
    }       
    
    int i_p_best, i_y_best, i_p_start, i_p_end, i_y_start, i_y_end;
    float radius_at_pitch;
    float f_i_p_best, f_i_y_best;

    // iterate over the accumulator
    for (i = 0; i < t_points.length(); i++) {
        marker_x = t_points[i].x;
        marker_y = t_points[i].y;
        marker_t = t_points[i].z;
        if (marker_t == 0) break;
        
        // possible implementation (similar to original, precomputed pitch index, fastest)
        if (marker_y > y + max_dist) {
            break;
        }
        
        dist = sqrt(float(pow2(x - marker_x) + pow2(y - marker_y)));
        if (dist > float(max_dist)) {
            continue;
        }

        yaw_angle = atan(float(marker_y - y), float(marker_x - x));
        yaw_overlap = int(abs(x - marker_x) <= int(dilation_radius) && abs(y - marker_y) <= int(dilation_radius));
        if (yaw_overlap != 0) {
            i_y_start = 0;
            i_y_end = int(PARAM_YAW_NUM)-1;
        } else {
            i_y_start = int(PARAM_YAW_NUM)-1;
            i_y_end = 0;
            for (i_y = 0; i_y < int(PARAM_YAW_NUM); i_y++) {
                if (abs(diff_wrap(yaw_angle, yaw_vals[i_y], float(PI))) < (0.75f * step_yaw)) {
                    if (i_y < i_y_start) i_y_start = i_y;
                    if (i_y > i_y_end) i_y_end = i_y;
                }
            }
        }

        pitch_angle = atan(float(marker_t), float(dist));
        i_p_best = int(min(int((pitch_angle - min_pitch) / step_pitch), PARAM_PITCH_NUM-1));
        i_p_start = int(max(0, i_p_best-1));
        i_p_end = int(min(PARAM_PITCH_NUM-1, i_p_best+1));

        for (i_p = i_p_start; i_p <= i_p_end; i_p++) {
            radius_at_pitch = float(marker_t) / tan(pitch_vals[i_p]);
            if (float(dist) >= float(radius_at_pitch - 1.0f) && float(dist) <= float(radius_at_pitch + 1.0f)) {
                for (i_y = i_y_start; i_y <= i_y_end; i_y++) {
                    increment_col_yp(i_y, i_p);
                }
            }
        }

        // alternatively, the following code calculates i_y and i_p ranges directly from angle values without trigonometry, overlaps based on similarity
        // it is unfinished and buggy!
        /*f_i_p_best = float(max(min(round(float(PITCH_OVERLAP_FRAC)*(pitch_angle - min_pitch) / step_pitch), float(PITCH_OVERLAP_FRAC)*float(PARAM_PITCH_NUM-1)), 0.0f)) / float(PITCH_OVERLAP_FRAC);
        i_p_start = max(int(ceil(f_i_p_best - 0.75f)), 0);
        i_p_end = min(int(floor(f_i_p_best + 0.75f)), int(PARAM_PITCH_NUM)-1);
        
        if (yaw_overlap != 0) {
            i_y_start = 0;
            i_y_end = int(PARAM_YAW_NUM)-1;
        } else {
            f_i_y_best = floor(mod(round(float(YAW_OVERLAP_FRAC) * yaw_angle / step_yaw), float(YAW_OVERLAP_FRAC)*float(PARAM_YAW_NUM))) / float(YAW_OVERLAP_FRAC);
            i_y_start = int(ceil(f_i_y_best - 0.75f));
            i_y_end = int(floor(f_i_y_best + 0.75f));
        }
        
        for (i_p = i_p_start; i_p <= i_p_end; i_p++) {
            for (i_y = i_y_start; i_y <= i_y_end; i_y++) {
                increment_col_yp(int(mod(float(i_y), float(PARAM_YAW_NUM))), i_p);
            }
        }*/
    }
    
    // merge with current flattened hough space
    ivec2 image_loc = ivec2(x, y) + tex_loc.xy;
    uint proj_data = imageLoad(hough_space_flattened, image_loc).r;
    uint prev_max_val = proj_data & uint(0x000000FF);
    uint prev_max_i_y = (proj_data & uint(0x0000FF00)) >> uint(8);
    uint prev_max_i_p = (proj_data & uint(0x00FF0000)) >> uint(16);
    uint prev_run_idx = (proj_data & uint(0xFF000000)) >> uint(24);
    bool store = false;
    if (prev_run_idx == uint(tex_loc.z) && prev_max_i_y == uint(max_i_y) && prev_max_i_p == uint(max_i_p)) {
        proj_data = (proj_data & ~uint(0x000000FF)) | (uint(max_val) + prev_max_val);
        imageStore(hough_space_flattened, image_loc, uvec4(proj_data));
    } else {
        if (uint(max_val) > prev_max_val || prev_run_idx != uint(tex_loc.z)) {
            proj_data = (uint(max_val) & uint(0x000000FF)) | ((uint(max_i_y) & uint(0x000000FF)) << uint(8)) | ((uint(max_i_p) & uint(0x000000FF)) << uint(16)) | ((uint(tex_loc.z) & uint(0x000000FF)) << uint(24));
            imageStore(hough_space_flattened, image_loc, uvec4(proj_data));
        }
    }
}
)"""";
};

} //namespace uvdar

#endif // HT4D_GPU_H
