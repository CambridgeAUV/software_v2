#pragma once

#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <cstdint>

namespace cauv {

template <typename M>
class MessageInbox {
    public:
    void subscribe(ros::NodeHandle &h, const std::string &topic, uint32_t queue_size) {
        sub = h.subscribe(topic, queue_size, &MessageInbox<M>::onMessage, this);
    };
    typename M::Ptr msg;
    operator bool() const { return (bool)msg; };
    private:
    void onMessage(const typename M::Ptr &m) {
        msg = m;
    };
    ros::Subscriber sub;
};

}
