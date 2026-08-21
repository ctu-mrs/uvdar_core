#include "uvdar_core/helpers/thread_pool.hpp"

#include <stdexcept>

namespace uvdar_core::helpers {

/**
 * @brief Construct fixed-size worker pool.
 */
ThreadPool::ThreadPool(std::size_t thread_count)
{
    if (thread_count == 0) {
        throw std::runtime_error("thread_pool_size must be greater than zero.");
    }

    workers_.reserve(thread_count);
    for (std::size_t index = 0; index < thread_count; ++index) {
        workers_.emplace_back([this]() { runWorker(); });
    }
}

/**
 * @brief Stop all workers and join threads.
 */
ThreadPool::~ThreadPool()
{
    {
        std::scoped_lock lock(mutex_);
        stop_ = true;
    }
    condition_.notify_all();

    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
}

/**
 * @brief Push task into queue and wake one worker.
 */
void ThreadPool::enqueue(std::function<void()> task)
{
    {
        std::scoped_lock lock(mutex_);
        tasks_.push(std::move(task));
    }
    condition_.notify_one();
}

/**
 * @brief Worker loop: wait for and execute queued tasks.
 */
void ThreadPool::runWorker()
{
    while (true) {
        std::function<void()> task;
        {
            std::unique_lock lock(mutex_);
            condition_.wait(lock, [this]() { return stop_ || !tasks_.empty(); });

            if (stop_ && tasks_.empty()) {
                return;
            }

            task = std::move(tasks_.front());
            tasks_.pop();
        }

        task();
    }
}

} // namespace uvdar_core::helpers
