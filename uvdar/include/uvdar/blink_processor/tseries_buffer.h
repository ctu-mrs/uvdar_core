#pragma once

#include <mutex>
#include <uvdar/blink_processor/marker_types.h>

namespace uvdar::blink_processor {

struct TseriesBuffer {
  std::vector<SeqPtr> buffer;
  mutable std::mutex mtx;
};

} // namespace uvdar::blink_processor