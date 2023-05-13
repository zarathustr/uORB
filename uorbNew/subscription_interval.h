#pragma once

#include <uORB/uorbNew/abs_time.h>
#include <uORB/uorbNew/subscription.h>
#include <uORB/uorbNew/uorb.h>

namespace uorb {

template <const orb_metadata &T>
class SubscriptionInterval : public Subscription<T> {
  using Type = typename msg::TypeMap<T>::type;

 private:
  template <typename Tp>
  constexpr Tp constrain(Tp val, Tp min_val, Tp max_val) const {
    return (val < min_val) ? min_val : ((val > max_val) ? max_val : val);
  }

 public:
  /**
   * Constructor
   *
   * @param meta The uORB metadata (usually from the ORB_ID() macro) for the
   * topic.
   * @param interval The requested maximum update interval in microseconds.
   * @param instance The instance for multi sub.
   */
  explicit SubscriptionInterval(uint32_t interval_us = 0,
                                uint8_t instance = 0) noexcept
      : Subscription<T>(instance), interval_us_(interval_us) {}

  ~SubscriptionInterval() = default;

  /**
   * Check if there is a new update.
   * */
  bool Updated() override {
    return Subscription<T>::Updated() &&
           (orb_elapsed_time_us(last_update_) >= interval_us_);
  }

  /**
   * Copy the struct if updated.
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was updated and copied successfully.
   */
  bool Update(Type *dst) override {
    if (Updated()) {
      return Copy(dst);
    }

    return false;
  }

  /**
   * Copy the struct
   * @param dst The destination pointer where the struct will be copied.
   * @return true only if topic was copied successfully.
   */
  bool Copy(Type *dst) override {
    if (Subscription<T>::Copy(dst)) {
      const orb_abstime_us now = orb_absolute_time_us();
      // shift last update time forward, but don't let it get further behind
      // than the interval
      last_update_ =
          constrain(last_update_ + interval_us_, now - interval_us_, now);
      return true;
    }

    return false;
  }

  uint32_t interval_us() const { return interval_us_; }
  uint32_t interval_ms() const { return interval_us_ / 1000; }
  void set_interval_us(uint32_t interval) { interval_us_ = interval; }
  void set_interval_ms(uint32_t interval) { interval_us_ = interval * 1000; }

 protected:
  orb_abstime_us last_update_{0};  // last update in microseconds
  uint32_t interval_us_{0};        // maximum update interval in microseconds
};

}  // namespace uorb
