// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_UTIL_THREADING_
#define COLMAP_SRC_UTIL_THREADING_

#include <atomic>
#include <climits>
#include <functional>
#include <future>
#include <list>
#include <queue>
#include <thread>
#include <unordered_map>

#include "util/timer.h"

namespace colmap {

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wkeyword-macro"
#endif

#ifdef __clang__
#pragma clang diagnostic pop  // -Wkeyword-macro
#endif

// Helper class to create single threads with simple controls and timing, e.g.:
//
//      class MyThread : public Thread {
//        enum {
//          PROCESSED_CALLBACK,
//        };
//
//        MyThread() { RegisterCallback(PROCESSED_CALLBACK); }
//        void Run() {
//          // Some setup routine... note that this optional.
//          if (setup_valid) {
//            SignalValidSetup();
//          } else {
//            SignalInvalidSetup();
//          }
//
//          // Some pre-processing...
//          for (const auto& item : items) {
//            BlockIfPaused();
//            if (IsStopped()) {
//              // Tear down...
//              break;
//            }
//            // Process item...
//            Callback(PROCESSED_CALLBACK);
//          }
//        }
//      };
//
//      MyThread thread;
//      thread.AddCallback(MyThread::PROCESSED_CALLBACK, []() {
//        std::cout << "Processed item"; })
//      thread.AddCallback(MyThread::STARTED_CALLBACK, []() {
//        std::cout << "Start"; })
//      thread.AddCallback(MyThread::FINISHED_CALLBACK, []() {
//        std::cout << "Finished"; })
//      thread.Start();
//      // thread.CheckValidSetup();
//      // Pause, resume, stop, ...
//      thread.Wait();
//      thread.Timer().PrintElapsedSeconds();
//

// note: 线程
class Thread {
 public:
  enum {
    STARTED_CALLBACK = INT_MIN,
    FINISHED_CALLBACK,
  };

  Thread();
  virtual ~Thread() = default;

  virtual void Start();   // api: 控制线程启动
  virtual void Stop();    // api: 控制线程停止
  virtual void Pause();   // api: 控制线程暂停
  virtual void Resume();  // api: 控制线程恢复
  virtual void Wait();    // api: 控制线程等待连接

  // api: 检查线程状态
  bool IsStarted();
  bool IsStopped();
  bool IsPaused();
  bool IsRunning();
  bool IsFinished();

  // To be called from inside the main run function. This blocks the main
  // caller, if the thread is paused, until the thread is resumed.
  // api: 内部调用，如果暂停，调用会阻塞回调
  void BlockIfPaused();

  // To be called from outside. This blocks the caller until the thread is
  // setup, i.e. it signaled that its setup was valid or not. If it never gives
  // this signal, this call will block the caller infinitely. Check whether
  // setup is valid. Note that the result is only meaningful if the thread gives
  // a setup signal.
  // api: 检查线程是否启动，若没有给出setup信号，会阻塞
  bool CheckValidSetup();

  // Set callbacks that can be triggered within the main run function.
  /**
   * \brief // api: 新增对应id回调的执行函数
   *
   * \param id 回调id
   * \param func 对应的执行函数
   */
  void AddCallback(const int id, const std::function<void()>& func);

  // api: 获取计时器，用于记录暂停的时间
  const Timer& GetTimer() const;

 protected:
  // api: 需要子类实现的纯虚函数
  // note: loop中需要暂停，调用BlockIfPaused；
  // note: 停止loop需要检查IsStopped状态并提前return
  virtual void Run() = 0;

  /**
   * \brief // api: 注册新的回调
   * // note: 只有注册的可以被set/reset以及在线程中被调用，线程构造后需要调用
   *
   * \param id 回调id
   */
  void RegisterCallback(const int id);

  // api: 执行对应id回调的函数，如果存在
  void Callback(const int id) const;

  // api: 获取线程id
  std::thread::id GetThreadId() const;

  // Signal that the thread is setup. Only call this function once.
  // api: 线程启动的信号，只需调用一次
  void SignalValidSetup();
  void SignalInvalidSetup();

 private:
  // Wrapper around the main run function to set the finished flag.
  // api: 封装主Run()函数，设置完成标志位
  void RunFunc();

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable pause_condition_;
  std::condition_variable setup_condition_;

  Timer timer_;

  bool started_;
  bool stopped_;
  bool paused_;
  bool pausing_;
  bool finished_;
  bool setup_;
  bool setup_valid_;

  std::unordered_map<int, std::list<std::function<void()>>> callbacks_;
};

// A thread pool class to submit generic tasks (functors) to a pool of workers:
//
//    ThreadPool thread_pool;
//    thread_pool.AddTask([]() { /* Do some work */ });
//    auto future = thread_pool.AddTask([]() { /* Do some work */ return 1; });
//    const auto result = future.get();
//    for (int i = 0; i < 10; ++i) {
//      thread_pool.AddTask([](const int i) { /* Do some work */ });
//    }
//    thread_pool.Wait();
//
// note: 线程池
class ThreadPool {
 public:
  static const int kMaxNumThreads = -1;

  explicit ThreadPool(const int num_threads = kMaxNumThreads);
  ~ThreadPool();

  // api: 线程池中的线程数
  inline size_t NumThreads() const;

  // Add new task to the thread pool.
  /**
   * \brief // api: 新增任务
   * 
   * \tparam func_t 
   * \tparam args_t 
   * \param f 函数
   * \param args 参数
   * \return std::future<typename std::result_of<func_t(args_t...)>::type> 
   */
  template <class func_t, class... args_t>
  auto AddTask(func_t&& f, args_t&&... args)
      -> std::future<typename std::result_of<func_t(args_t...)>::type>;

  // Stop the execution of all workers.
  // api: 停止
  void Stop();

  // Wait until tasks are finished.
  // api: 等待直到所有任务结束
  void Wait();

  // Get the unique identifier of the current thread.
  // api: 获取当前线程id
  std::thread::id GetThreadId() const;

  // Get the index of the current thread. In a thread pool of size N,
  // the thread index defines the 0-based index of the thread in the pool.
  // In other words, there are the thread indices 0, ..., N-1.
  // api: 获取当前线程的索引号
  int GetThreadIndex();

 private:
  void WorkerFunc(const int index);

  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;

  std::mutex mutex_;
  std::condition_variable task_condition_;
  std::condition_variable finished_condition_;

  bool stopped_;
  int num_active_workers_;

  std::unordered_map<std::thread::id, int> thread_id_to_index_;
};

// A job queue class for the producer-consumer paradigm.
//
//    JobQueue<int> job_queue;
//
//    std::thread producer_thread([&job_queue]() {
//      for (int i = 0; i < 10; ++i) {
//        job_queue.Push(i);
//      }
//    });
//
//    std::thread consumer_thread([&job_queue]() {
//      for (int i = 0; i < 10; ++i) {
//        const auto job = job_queue.Pop();
//        if (job.IsValid()) { /* Do some work */ }
//        else { break; }
//      }
//    });
//
//    producer_thread.join();
//    consumer_thread.join();
//
// note: 工作队列
template <typename T>
class JobQueue {
 public:
  class Job {
   public:
    Job() : valid_(false) {}
    explicit Job(T data) : data_(std::move(data)), valid_(true) {}

    // api: Check whether the data is valid.
    bool IsValid() const { return valid_; }

    // api: Get reference to the data.
    T& Data() { return data_; }
    const T& Data() const { return data_; }

   private:
    T data_;
    bool valid_;
  };

  JobQueue();
  explicit JobQueue(const size_t max_num_jobs);
  ~JobQueue();

  // The number of pushed and not popped jobs in the queue.
  // api: 当前队列中的jobs数量
  size_t Size();

  // Push a new job to the queue. Waits if the number of jobs is exceeded.
  // api: 新增一个新的任务，如果超过限制就等待
  bool Push(T data);

  // Pop a job from the queue. Waits if there is no job in the queue.
  // api: 弹出一个任务，如果没有任务就等待
  Job Pop();

  // Wait for all jobs to be popped and then stop the queue.
  // api: 等待所有任务被弹出，停止
  void Wait();

  // Stop the queue and return from all push/pop calls with false.
  // api: 停止
  void Stop();

  // Clear all pushed and not popped jobs from the queue.
  // api: 清空所有未弹出的任务
  void Clear();

 private:
  size_t max_num_jobs_;
  std::atomic<bool> stop_;
  std::queue<T> jobs_;
  std::mutex mutex_;
  std::condition_variable push_condition_;
  std::condition_variable pop_condition_;
  std::condition_variable empty_condition_;
};

// Return the number of logical CPU cores if num_threads <= 0,
// otherwise return the input value of num_threads.
// api: 获取当前有效的线程数，若<=0则返回cpu的核心数
int GetEffectiveNumThreads(const int num_threads);

////////////////////////////////////////////////////////////////////////////////
// Implementation
////////////////////////////////////////////////////////////////////////////////

size_t ThreadPool::NumThreads() const { return workers_.size(); }

template <class func_t, class... args_t>
auto ThreadPool::AddTask(func_t&& f, args_t&&... args)
    -> std::future<typename std::result_of<func_t(args_t...)>::type> {
  typedef typename std::result_of<func_t(args_t...)>::type return_t;

  auto task = std::make_shared<std::packaged_task<return_t()>>(
      std::bind(std::forward<func_t>(f), std::forward<args_t>(args)...));

  std::future<return_t> result = task->get_future();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (stopped_) {
      throw std::runtime_error("Cannot add task to stopped thread pool.");
    }
    tasks_.emplace([task]() { (*task)(); });
  }

  task_condition_.notify_one();

  return result;
}

template <typename T>
JobQueue<T>::JobQueue() : JobQueue(std::numeric_limits<size_t>::max()) {}

template <typename T>
JobQueue<T>::JobQueue(const size_t max_num_jobs)
    : max_num_jobs_(max_num_jobs), stop_(false) {}

template <typename T>
JobQueue<T>::~JobQueue() {
  Stop();
}

template <typename T>
size_t JobQueue<T>::Size() {
  std::unique_lock<std::mutex> lock(mutex_);
  return jobs_.size();
}

template <typename T>
bool JobQueue<T>::Push(T data) {
  std::unique_lock<std::mutex> lock(mutex_);
  // step: 1 数量满&运行时，等待pop的cv通知解锁
  while (jobs_.size() >= max_num_jobs_ && !stop_) {
    pop_condition_.wait(lock);
  }

  // step: 2 若停止则返回false，否则加入新的job，push的cv通知解锁
  if (stop_) {
    return false;
  } else {
    jobs_.push(std::move(data));
    push_condition_.notify_one();
    return true;
  }
}

template <typename T>
typename JobQueue<T>::Job JobQueue<T>::Pop() {
  std::unique_lock<std::mutex> lock(mutex_);
  // step: 1 空&运行时，等待push的cv通知解锁
  while (jobs_.empty() && !stop_) {
    push_condition_.wait(lock);
  }

  // step: 2 若停止则返回false，否则弹出job，pop的cv通知解锁
  if (stop_) {
    return Job();
  } else {
    Job job(std::move(jobs_.front()));
    jobs_.pop();
    pop_condition_.notify_one();

    // step: 3 pop空需要empty的cv通知解锁
    if (jobs_.empty()) {
      empty_condition_.notify_all();
    }
    return job;
  }
}

template <typename T>
void JobQueue<T>::Wait() {
  std::unique_lock<std::mutex> lock(mutex_);
  // step: 非空时等待empty的cv通知解锁
  while (!jobs_.empty()) {
    empty_condition_.wait(lock);
  }
}

template <typename T>
void JobQueue<T>::Stop() {
  // step: stop_标志位置1，并不阻塞任何push/pop
  stop_ = true;
  push_condition_.notify_all();
  pop_condition_.notify_all();
}

template <typename T>
void JobQueue<T>::Clear() {
  std::unique_lock<std::mutex> lock(mutex_);
  std::queue<T> empty_jobs;
  std::swap(jobs_, empty_jobs);
}

}  // namespace colmap

#endif  // COLMAP_SRC_UTIL_THREADING_
