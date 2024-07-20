#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>

namespace tools
{
template <typename T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(
    size_t max_size, std::function<void(void)> full_handler = [] {})
  : max_size_(max_size), full_handler_(full_handler)
  {
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    if (queue_.size() >= max_size_) {
      full_handler_();
      return;
    }

    queue_.push(value);
    not_empty_condition_.notify_all();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);

    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });

    value = queue_.front();
    queue_.pop();
  }

private:
  std::queue<T> queue_;
  size_t max_size_;
  std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void(void)> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP