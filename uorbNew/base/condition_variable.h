//
// Copyright (c) 2021 shawnfeng. All rights reserved.
//
#pragma once

#include <pthread.h>
#include <stdint.h>
#if !defined(__APPLE__)
#include <time.h>
#else
#include <sys/time.h>
#endif

#include "uORB/uorbNew/base/mutex.h"


namespace uorb {
namespace base {

#if !defined(__APPLE__)
static inline
int _condvar_clock_gettime(int clock_type, timespec *ts)
{
    return clock_gettime(clock_type, ts);
}


#else // #if !HAVE_PTHREAD_CONDATTR_SETCLOCK
static inline
int _condvar_clock_gettime(int clock_type, timespec *ts)
{
    (void)clock_type; // Unused
    timeval tv;
    return gettimeofday(&tv, NULL) == 0 ? (ts->tv_sec = tv.tv_sec, ts->tv_nsec = tv.tv_usec * 1000, 0) : (-1);
}
#endif


template <clockid_t clock_id>
class ConditionVariable {
 public:
  ConditionVariable() noexcept {
    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
#if !defined(__APPLE__)
    pthread_condattr_setclock(&attr, (clock_id));
#endif
    pthread_cond_init((&cond_), &attr);
  }

  ~ConditionVariable() noexcept { pthread_cond_destroy(&cond_); }

 public:
  ConditionVariable(const ConditionVariable &) = delete;
  ConditionVariable &operator=(const ConditionVariable &) = delete;

  void notify_one() noexcept { pthread_cond_signal(&cond_); }

  void notify_all() noexcept { pthread_cond_broadcast(&cond_); }

  void wait(Mutex &lock) noexcept {  // NOLINT
    pthread_cond_wait(&cond_, lock.GetNativeHandle());
  }

  template <typename Predicate>
  void wait(Mutex &lock, Predicate p) {  // NOLINT
    while (!p()) wait(lock);
  }

  // Return true if successful
  bool wait_until(Mutex &lock, const struct timespec &atime) {  // NOLINT
    return pthread_cond_timedwait(&cond_, lock.GetNativeHandle(), &atime) == 0;
  }

  // Return true if successful
  template <typename Predicate>
  // NOLINTNEXTLINE
  bool wait_until(Mutex &lock, const struct timespec &atime, Predicate p) {
    // Not returned until timeout or other error
    while (!p())
      if (!wait_until(lock, atime)) return p();
    return true;
  }

  // Return true if successful
  bool wait_for(Mutex &lock, unsigned long time_ms) {  // NOLINT
    struct timespec atime {};
    GenerateFutureTime(clock_id, time_ms, &atime);
    return wait_until(lock, atime);
  }

  // Return true if successful
  template <typename Predicate>
  bool wait_for(Mutex &lock, unsigned long time_ms, Predicate p) {  // NOLINT
    struct timespec atime {};
    GenerateFutureTime(clock_id, time_ms, &atime);

    // Not returned until timeout or other error
    while (!p())
      if (!wait_until(lock, atime)) return p();
    return true;
  }

  pthread_cond_t *native_handle() { return &cond_; }

 private:
  // Increase time_ms time based on the current clockid time
  inline void GenerateFutureTime(clockid_t clockid, uint32_t time_ms,
                                 struct timespec *out_ptr) {
    if (!out_ptr) return;
    auto &out = *out_ptr;
    // Calculate an absolute time in the future
    const decltype(out.tv_nsec) kSec2Nsec = 1000 * 1000 * 1000;
    _condvar_clock_gettime(clockid, &out);
    out.tv_sec += time_ms / 1000;
    time_ms %= 1000;
    uint64_t nano_secs = out.tv_nsec + ((uint64_t)time_ms * 1000 * 1000);
    out.tv_nsec = nano_secs % kSec2Nsec;
    out.tv_sec += nano_secs / kSec2Nsec;
  }

  pthread_cond_t cond_{};
};

typedef ConditionVariable<CLOCK_MONOTONIC> MonoClockCond;
typedef ConditionVariable<CLOCK_REALTIME> RealClockCond;

template <clockid_t clock_id>
class SimpleSemaphore {
 public:
  explicit SimpleSemaphore(unsigned int count = 0) : count_(count) {}
  SimpleSemaphore(const SimpleSemaphore &) = delete;
  SimpleSemaphore &operator=(const SimpleSemaphore &) = delete;

  // increments the internal counter and unblocks acquirers
  void release() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    ++count_;
    // The previous semaphore was 0, and there may be waiting tasks
    if (count_ == 1) condition_.notify_one();
  }

  // decrements the internal counter or blocks until it can
  void acquire() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    condition_.wait(mutex_, [&] { return count_ > 0; });
    --count_;
  }

  // tries to decrement the internal counter without blocking
  bool try_acquire() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    if (count_) {
      --count_;
      return true;
    }
    return false;
  }

  // tries to decrement the internal counter, blocking for up to a duration time
  bool try_acquire_for(int time_ms) {
    LockGuard<decltype(mutex_)> lock(mutex_);
    bool finished =
        condition_.wait_for(mutex_, time_ms, [&] { return count_ > 0; });
    if (finished) --count_;
    return finished;
  }

  unsigned int get_value() {
    LockGuard<decltype(mutex_)> lock(mutex_);
    return count_;
  }

 private:
  // Hide this function in case the client is not sure which clockid to use
  // tries to decrement the internal counter, blocking until a point in time
  bool try_acquire_until(const struct timespec &atime) {
    LockGuard<decltype(mutex_)> lock(mutex_);
    bool finished =
        condition_.wait_until(mutex_, atime, [&] { return count_ > 0; });
    if (finished) --count_;
    return finished;
  }

  Mutex mutex_;
  ConditionVariable<clock_id> condition_;
  unsigned int count_;
};

typedef SimpleSemaphore<CLOCK_REALTIME> RealClockSemaphore;
typedef SimpleSemaphore<CLOCK_MONOTONIC> MonoClockSemaphore;

}  // namespace base
}  // namespace uorb
