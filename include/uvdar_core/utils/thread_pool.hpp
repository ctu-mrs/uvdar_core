#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace uvdar_core::utils {

class ThreadPool {
public:
    /**
     * @brief Construct worker pool of given size.
     * @param thread_count Number of worker threads.
     */
    explicit ThreadPool(std::size_t thread_count);
    /**
     * @brief Stop workers and join threads.
     */
    ~ThreadPool();

    /**
     * @brief Queue one task for execution.
     */
    void enqueue(std::function<void()> task);

private:
    /**
     * @brief Worker loop that pulls and executes queued tasks.
     */
    void runWorker();

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> tasks_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool stop_ = false;
};

} // namespace uvdar_core::utils
