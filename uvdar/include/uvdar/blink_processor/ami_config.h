#pragma once

#include <opencv2/opencv.hpp>

namespace uvdar {

/* AmiConfig //{ */
struct AmiConfig {
  cv::Point max_px_shift{2, 2};
  int max_zeros_consecutive{10};
  int stored_seq_len_factor{20};
  int max_buffer_length{1000};
  int frame_length{16};
  int poly_order{4};
  float decay_factor{0.1f};
  double conf_probab_percent{75.0};
  int allowed_BER_per_seq{0};
  int loaded_var_pub_rate{20};
  double draw_predict_window_sec{0.0};
};
//}

} // namespace uvdar
