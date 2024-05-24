#ifndef UV_LED_FAST_GPU_H
#define UV_LED_FAST_GPU_H

extern "C" {
#include "../compute_lib/compute_lib.h"
}
#include "uv_led_detect_fast.h"

typedef struct {
    uint16_t y;
    uint16_t x;
} fast_det_pt_t;

namespace uvdar {

  class UVDARLedDetectFASTGPU : public UVDARLedDetectFAST {
    public:
      UVDARLedDetectFASTGPU(bool i_gui, bool i_debug, int i_threshold, int i_threshold_diff, int i_threshold_sun, std::vector<cv::Mat> i_masks);
      ~UVDARLedDetectFASTGPU();
      bool processImage(const cv::Mat i_image, std::vector<cv::Point2i>& detected_points, std::vector<cv::Point2i>& sun_points, int mask_id=-1);
      bool initDelayed(const cv::Mat i_image);

    private:
      void init();
      uint32_t cpuFindMarkerCentroids(fast_det_pt_t* markers, uint32_t init_cnt, uint32_t distance_px, std::vector<cv::Point2i>& detected_points);

      bool initialized_ = false;
      bool first_ = true;

      bool use_masks = true;

      bool                   m_lines_;
      int                    m_accumLength_;

      cv::Mat image_curr_;
      cv::Mat  image_view_;

      compute_lib_instance_t compute_inst;
      compute_lib_program_t compute_prog;
      compute_lib_image2d_t texture_in, mask;
      compute_lib_acbo_t markers_count_acbo, sun_pts_count_acbo;
      compute_lib_ssbo_t markers_ssbo, sun_pts_ssbo;

      uint32_t local_size_x;
      uint32_t local_size_y;
      cv::Size image_size;
      uint32_t max_markers_count;
      uint32_t max_sun_pts_count;

      static constexpr char fastlike_shader_src[] = R""""(
#version 310 es

#define LOCAL_SIZE_X %d
#define LOCAL_SIZE_Y %d
#define FAST_THRESHOLD %d
#define FAST_THRESHOLD_DIFF %d
#define FAST_THRESHOLD_SUN %d
#define MAX_MARKERS_COUNT %d
#define MAX_SUN_PTS_COUNT %d

layout (local_size_x = LOCAL_SIZE_X, local_size_y = LOCAL_SIZE_Y, local_size_z = 1) in;

layout(rgba8ui, binding = 0) readonly uniform highp uimage2D image_in;

layout(rgba8ui, binding = 1) readonly uniform highp uimage2D mask;

layout(binding = 2, offset = 0) uniform atomic_uint sun_pts_count;
layout(binding = 3, offset = 0) uniform atomic_uint markers_count;

layout(std430, binding = 4) buffer markers_buffer { uint markers[];};
layout(std430, binding = 5) buffer sun_pts_buffer { uint sun_pts[];};

ivec2 image_size = ivec2(-1, -1);
ivec2 center_pos = ivec2(-1, -1);
int center_val = -1;

#define FAST_RESULT_NONE 0
#define FAST_RESULT_MARKER 1
#define FAST_RESULT_SUN 2

#define BRESENHAM_EMIT_PY_MX 0
#define BRESENHAM_EMIT_MY_PX 1
#define BRESENHAM_EMIT_PX_MY 2
#define BRESENHAM_EMIT_MX_PY 3
#define BRESENHAM_EMIT_PY_PX 4
#define BRESENHAM_EMIT_MY_MX 5
#define BRESENHAM_EMIT_PX_PY 6
#define BRESENHAM_EMIT_MX_MY 7
#define BRESENHAM_UPDATE_X_Y 8

int x, y, P, state;

void bresenham_circle_init(int radius)
{
    x = 0;
    y = radius;
    P = 3 - 2*radius;
    state = BRESENHAM_EMIT_PY_MX;
}

ivec2 bresenham_circle_next_pt()
{
    if (state == BRESENHAM_UPDATE_X_Y) {
        if (P < 0) {
            x += 1;
            P += 4*x + 6;
        } else {
            x += 1;
            y -= 1;
            P += 4*(x - y) + 10;
        }
        state = BRESENHAM_EMIT_PY_MX;
    }
    
    if (x <= y) {
        switch (state) {
            case BRESENHAM_EMIT_PY_MX:
                state = BRESENHAM_EMIT_MY_PX;
                return center_pos + ivec2(+y, -x);
            case BRESENHAM_EMIT_MY_PX:
                if (x < y) {
                    state = BRESENHAM_EMIT_PX_MY;
                } else if (x > 0) {
                    state = BRESENHAM_EMIT_PY_PX;
                } else {
                    state = BRESENHAM_UPDATE_X_Y;
                }
                return center_pos + ivec2(-y, +x);
            case BRESENHAM_EMIT_PX_MY:
                state = BRESENHAM_EMIT_MX_PY;
                return center_pos + ivec2(+x, -y);
            case BRESENHAM_EMIT_MX_PY:
                if (x > 0) {
                    state = BRESENHAM_EMIT_PY_PX;
                } else {
                    state = BRESENHAM_UPDATE_X_Y;
                }
                return center_pos + ivec2(-x, +y);
            case BRESENHAM_EMIT_PY_PX:
                state = BRESENHAM_EMIT_MY_MX;
                return center_pos + ivec2(+y, +x);
            case BRESENHAM_EMIT_MY_MX:
                if (x < y) {
                    state = BRESENHAM_EMIT_PX_PY;
                } else {
                    state = BRESENHAM_UPDATE_X_Y;
                }
                return center_pos + ivec2(-y, -x);
            case BRESENHAM_EMIT_PX_PY:
                state = BRESENHAM_EMIT_MX_MY;
                return center_pos + ivec2(+x, +y);
            case BRESENHAM_EMIT_MX_MY:
                state = BRESENHAM_UPDATE_X_Y;
                return center_pos + ivec2(-x, -y);
        }
    } else {
        return ivec2(-1, -1);
    }
}
        
int run_fast(int radius)
{
    int val;
    int boundary_max = 0x00;
    int boundary_min = 0xFF;
    ivec2 pos;

    bresenham_circle_init(radius);
    pos = bresenham_circle_next_pt();

    while (pos.x >= 0 && pos.x < image_size.x && pos.y >= 0 && pos.y < image_size.y) {
        val = int(imageLoad(image_in, pos).r);
        if (val > boundary_max) { boundary_max = val; }
        if (val < boundary_min) { boundary_min = val; }
        pos = bresenham_circle_next_pt();
    }
    if (!((pos.x == -1) && (pos.y == -1)))
      return FAST_RESULT_NONE;
    
    if ((center_val - boundary_max) >= FAST_THRESHOLD_DIFF) {
        return FAST_RESULT_MARKER;
    } else if (center_val >= FAST_THRESHOLD_SUN && (center_val - boundary_min) <= FAST_THRESHOLD_DIFF) {
        return FAST_RESULT_SUN;
    } else {
        return FAST_RESULT_NONE;
    }
}

void main()
{
    if (atomicCounter(markers_count) >= uint(MAX_MARKERS_COUNT)) {
        return;
    }

    image_size = imageSize(image_in);
    center_pos = ivec2(gl_GlobalInvocationID.xy);

    center_val = int((imageLoad(image_in, center_pos).r) & (imageLoad(mask, center_pos).r));
    
    if (center_val >= FAST_THRESHOLD) {
        switch (run_fast(3)) {
            case FAST_RESULT_MARKER:
                markers[atomicCounterIncrement(markers_count)] = ((uint(center_pos.x) & uint(0x0000FFFF)) << 16) | (uint(center_pos.y) & uint(0x0000FFFF));
                break;
            case FAST_RESULT_SUN:
                if (atomicCounter(sun_pts_count) < uint(MAX_SUN_PTS_COUNT))
                  sun_pts[atomicCounterIncrement(sun_pts_count)] = ((uint(center_pos.x) & uint(0x0000FFFF)) << 16) | (uint(center_pos.y) & uint(0x0000FFFF));
                break;
            case FAST_RESULT_NONE:
                switch (run_fast(4)) {
                    case FAST_RESULT_MARKER:
                        markers[atomicCounterIncrement(markers_count)] = ((uint(center_pos.x) & uint(0x0000FFFF)) << 16) | (uint(center_pos.y) & uint(0x0000FFFF));
                        break;
                    case FAST_RESULT_SUN:
                        if (atomicCounter(sun_pts_count) < uint(MAX_SUN_PTS_COUNT))
                          sun_pts[atomicCounterIncrement(sun_pts_count)] = ((uint(center_pos.x) & uint(0x0000FFFF)) << 16) | (uint(center_pos.y) & uint(0x0000FFFF));
                        break;
                    case FAST_RESULT_NONE:
                        break;
                }
                break;
        }
    } 
}
)"""";
  };
}


#endif  // UV_LED_FAST_GPU_H
