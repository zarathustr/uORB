/* Author: Jin Wu, Hong Kong University of Science and Technology (HKUST)
   E-mail: jin_wu_uestc@hotmail.com
   Website: https://zarathustr.github.io
*/


#include <thread>
#include <unistd.h>
#include <cstring>

#include <uORB/uORB.h>
#include "uORB/uorbNew/abs_time.h"
#include "uORB/uorbNew/publication.h"
#include "uORB/uorbNew/publication_multi.h"
#include "uORB/uorbNew/subscription.h"
#include "uORB/uorbNew/subscription_interval.h"
#include <uORB/uorbNew/uorb.h>

static void * orb_subscriber_handles[255] = {};
static int orb_subscriber_nums = 0;

orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data)
{
    return __orb_create_publication(meta);
}

orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
                                 int priority)
{
    return __orb_create_publication_multi(meta, (unsigned int *)instance);
}

int orb_unadvertise(orb_advert_t &handle)
{
    void *handle__ = handle;
    if(__orb_destroy_publication((orb_publication_t **)&handle__))
        return 0;
    return -1;
}

int	orb_check(int handle, bool *updated)
{
    if(__orb_check_update((orb_subscription_t *) orb_subscriber_handles[handle])) {
        *updated = true;
        return 0;
    }
    *updated = false;
    return -1;
}

int	orb_copy(const struct orb_metadata *meta, int handle, void *buffer)
{
    if(__orb_copy((orb_subscription_t *) orb_subscriber_handles[handle], buffer))
        return 0;
    return -1;
}

int	orb_subscribe(const struct orb_metadata *meta)
{
    return orb_subscribe_multi(meta, 0);
}

int	orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
    auto ret = __orb_create_subscription_multi(meta, instance);
    orb_subscriber_handles[orb_subscriber_nums++] = ret;
    return orb_subscriber_nums - 1;
}

int	orb_unsubscribe(int handle)
{
    orb_subscription_t *array[] = {(orb_subscription_t *) orb_subscriber_handles[handle]};
    if(__orb_destroy_subscription(array))
        return 0;
    return -1;
}

int	orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
    if(__orb_publish((orb_publication_t *) handle, data))
        return 0;
    return -1;
}

int orb_publish_auto(const struct orb_metadata *meta, orb_advert_t *handle, const void *data, int *instance,
                     int priority)
{
    if(__orb_publish_auto(meta, (orb_publication_t **) handle, data, (unsigned int *)instance))
        return 0;
    return -1;
}

int orb_poll(struct pollfd *fds, nfds_t nfds, int timeout)
{
    orb_pollfd *fds_ = new orb_pollfd[nfds];
    for(int i = 0; i < nfds; ++i)
    {
        fds_[i].fd = (orb_subscription_t *)orb_subscriber_handles[fds[i].fd];
        fds_[i].events = fds[i].events;
        fds_[i].revents = fds[i].revents;
    }
    int ret = __orb_poll(fds_, nfds, timeout);
    delete [] fds_;
    return ret;
}

int	orb_set_interval(int handle, unsigned interval)
{

}

int	orb_get_interval(int handle, unsigned *interval){
    *interval = 1000;
}