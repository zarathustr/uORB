/* Author: Jin Wu, Hong Kong University of Science and Technology (HKUST)
   E-mail: jin_wu_uestc@hotmail.com
   Website: https://zarathustr.github.io
*/


#ifndef _UORB_UORB_H
#define _UORB_UORB_H

/**
 * @file uORB.h
 * API for the uORB lightweight object broker.
 */
#pragma once

#include <ctype.h>
#include <stdint.h>
#include <cstdint>
#include <stdbool.h>
#include <stddef.h>
#include <poll.h>

#define pollfd_struct_t         pollfd

enum ORB_PRIO {
    ORB_PRIO_MIN = 1, // leave 0 free for other purposes, eg. marking an uninitialized value
    ORB_PRIO_VERY_LOW = 25,
    ORB_PRIO_LOW = 50,
    ORB_PRIO_DEFAULT = 75,
    ORB_PRIO_HIGH = 100,
    ORB_PRIO_VERY_HIGH = 125,
    ORB_PRIO_MAX = 255
};

inline int orb_prios(int i)
{
    switch(i)
    {
        case 0:
            return ORB_PRIO_MIN;
        case 1:
            return ORB_PRIO_VERY_LOW;
        case 2:
            return ORB_PRIO_LOW;
        case 3:
            return ORB_PRIO_DEFAULT;
        case 4:
            return ORB_PRIO_HIGH;
        case 5:
            return ORB_PRIO_VERY_HIGH;
        case 6:
            return ORB_PRIO_MAX;
        default:
            return ORB_PRIO_DEFAULT;
    }
}


#ifdef __cplusplus
extern "C" {
#endif

typedef void 	*orb_advert_t;

/**
 * @see uORB::Manager::orb_advertise()
 */
extern orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data) ;

/**
 * @see uORB::Manager::orb_advertise()
 */
extern orb_advert_t orb_advertise_queue(const struct orb_metadata *meta, const void *data,
					unsigned int queue_size) ;

/**
 * @see uORB::Manager::orb_advertise_multi()
 */
extern orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
					int priority) ;

/**
 * @see uORB::Manager::orb_advertise_multi()
 */
extern orb_advert_t orb_advertise_multi_queue(const struct orb_metadata *meta, const void *data, int *instance,
		int priority, unsigned int queue_size) ;

/**
 * @see uORB::Manager::orb_unadvertise()
 */
extern int orb_unadvertise(orb_advert_t &handle) ;

/**
 * Advertise as the publisher of a topic.
 *
 * This performs the initial advertisement of a topic; it creates the topic
 * node in /obj if required and publishes the initial data.
 *
 * @see uORB::Manager::orb_advertise_multi() for meaning of the individual parameters
 */
extern int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data, int *instance,
			    int priority);

/**
 * @see uORB::Manager::orb_publish()
 */
extern int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) ;

/**
 * @see uORB::Manager::orb_subscribe()
 */
extern int	orb_subscribe(const struct orb_metadata *meta) ;

/**
 * @see uORB::Manager::orb_subscribe_multi()
 */
extern int	orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance) ;

/**
 * @see uORB::Manager::orb_unsubscribe()
 */
extern int	orb_unsubscribe(int handle) ;

/**
 * @see uORB::Manager::orb_copy()
 */
extern int	orb_copy(const struct orb_metadata *meta, int handle, void *buffer) ;

/**
 * @see uORB::Manager::orb_check()
 */
extern int	orb_check(int handle, bool *updated) ;

/**
 * @see uORB::Manager::orb_stat()
 */
extern int	orb_stat(int handle, uint64_t *time) ;

/**
 * @see uORB::Manager::orb_exists()
 */
extern int	orb_exists(const struct orb_metadata *meta, int instance) ;

/**
 * @see uORB::Manager::orb_priority()
 */
extern int	orb_priority(int handle, int32_t *priority) ;

/**
 * @see uORB::Manager::orb_set_interval()
 */
extern int	orb_set_interval(int handle, unsigned interval) ;

/**
 * @see uORB::Manager::orb_get_interval()
 */
extern int	orb_get_interval(int handle, unsigned *interval) ;

int orb_poll(struct pollfd *fds, nfds_t nfds, int timeout);

#ifdef __cplusplus
}
#endif

#endif /* _UORB_UORB_H */
