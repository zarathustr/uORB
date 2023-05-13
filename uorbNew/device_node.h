#pragma once

#include <uORB/uorbNew/internal/noncopyable.h>
#include <uORB/uorbNew/uorb.h>

#include <cerrno>
#include <set>

#include "base/condition_variable.h"
#include "base/intrusive_list.h"
#include "base/mutex.h"
#include "callback.h"

namespace uORBTest {
class UnitTest;
}

namespace uorb {
class DeviceMaster;

/**
 * Per-object device instance.
 */
class DeviceNode : public ListNode<DeviceNode *>,
                   private internal::Noncopyable {
  friend DeviceMaster;

 public:
  // Publish a data to this node.
  bool Publish(const void *data);

  void add_subscriber();
  void remove_subscriber();
  uint8_t subscriber_count() const { return subscriber_count_; }
  bool has_anonymous_subscriber() const { return has_anonymous_subscriber_; }
  void mark_anonymous_subscriber() { has_anonymous_subscriber_ = true; }

  void add_publisher();
  void remove_publisher();
  uint8_t publisher_count() const { return publisher_count_; }
  bool has_anonymous_publisher() const { return has_anonymous_publisher_; }
  void mark_anonymous_publisher() { has_anonymous_publisher_ = true; }

  // Whether meta and instance are the same as the current one
  inline bool IsSameWith(const orb_metadata &meta, uint8_t instance) const {
    return IsSameWith(meta) && (instance_ == instance);
  }

  inline bool IsSameWith(const orb_metadata &meta) const {
    return &meta_ == &meta;
  }

  // add item to list of work items to schedule on node update
  template <typename Callback>
  bool RegisterCallback(Callback *callback) {
    if (!callback) {
      errno = EINVAL;
      return false;
    }

    uorb::base::LockGuard<base::Mutex> lg(lock_);
    uorb::DeviceNode::callbacks_.emplace(callback);
    return true;
  }

  // remove item from list of work items
  template <typename Callback>
  bool UnregisterCallback(Callback *callback) {
    base::LockGuard<base::Mutex> lg(lock_);
    return uorb::DeviceNode::callbacks_.erase(callback) != 0;
  }

  // Returns the number of updated data relative to the parameter 'generation'
  unsigned updates_available(unsigned generation) const;
  unsigned initial_generation() const;

  unsigned queue_size() const { return queue_size_; }

  const char *name() const { return meta_.o_name; }
  uint8_t instance() const { return instance_; }

  /**
   * Copies data and the corresponding generation
   * from a node to the buffer provided.
   *
   * @param dst
   *   The buffer into which the data is copied.
   * @param sub_generation
   *   The generation that was copied.
   * @return bool
   *   Returns true if the data was copied.
   */
  bool Copy(void *dst, unsigned *sub_generation) const;

 private:
  friend uORBTest::UnitTest;

  const orb_metadata &meta_; /**< object metadata information */
  const uint8_t instance_;   /**< orb multi instance identifier */

  uint8_t *data_{nullptr};    /**< allocated object buffer */
  const uint32_t queue_size_; /**< maximum number of elements in the queue */
  uint32_t generation_{0};    /**< object generation count */

  mutable base::Mutex lock_{};

  uint8_t subscriber_count_{0};
  bool has_anonymous_subscriber_{false};
  uint8_t publisher_count_{0};
  bool has_anonymous_publisher_{false};

  std::set<detail::CallbackBase *> callbacks_;

  DeviceNode(const struct orb_metadata &meta, uint8_t instance);
  ~DeviceNode();
};
}  // namespace uorb
