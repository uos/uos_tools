import threading
import rospy
from message_filters import SimpleFilter

class SimpleApproximateTimeSynchronizer(SimpleFilter):
    """
    Synchronize multiple topics using a simple
    and non-optimal strategy.

    This caches messages for all subscribed filters
    and passes them on (bundled) whenever there are
    messages available on all topics.
    """

    def __init__(self, filters, queue_size=10, max_difference=rospy.Duration(.5)):
        """
        :param queue_size: The size of the message buffer for each filter
        :param max_difference: The maximal time difference between messages in the same bundle
        """

        SimpleFilter.__init__(self)
        self.connectInput(filters)

        self.queue_size= queue_size
        self.max_diff = max_difference
        self.lock = threading.Lock()

    def connectInput(self, filters):

        self.queues = [[] for f in filters]
        self.input_connections = [f.registerCallback(self.add, q) for (f, q) in zip(filters, self.queues)]

    def add(self, msg, queue):

        self.lock.acquire()
        queue.append(msg)

        while len(queue) > self.queue_size:
            queue.pop(0)

        if len(queue) == 1 and not([] in self.queues):
            # this was the last missing message to fire a synchronized msg

            t= queue[0].header.stamp

            def mindiff(queue, time):
                """ return the message closest to time """
                selmsg=queue[0]
                for msg in queue[1:]:
                    if abs(msg.header.stamp - time) < abs(selmsg.header.stamp - time):
                        selmsg= msg
                    elif abs(msg.header.stamp - time) > abs(selmsg.header.stamp - time):
                        break
                return selmsg

            # this gets all messages _centered_ around the one which arrived last
            # _NOT_ the best match as the C++ ApproximateTime policy
            msgs= [mindiff(q, t) for q in self.queues]
            times= [m.header.stamp for m in msgs]

            bundle_size= max(times)-min(times)
            if bundle_size < self.max_diff:
                self.signalMessage(*msgs)
            else:
                rospy.loginfo("discard bundle because overall time difference %s > %s" % (str(bundle_size), str(self.max_diff)))

            # cleanup all messages before and including those just sent
            for (q,t) in zip(self.queues, times):
                while len(q) > 0 and q[0].header.stamp <= t:
                    q.pop(0)

        self.lock.release()
