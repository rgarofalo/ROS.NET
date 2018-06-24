using Messages;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class MessageAndSerializerFunc
    {
        internal RosMessage msg;
        internal bool nocopy;
        internal TopicManagerAsync.SerializeFunc serfunc;
        internal bool serialize;

        internal MessageAndSerializerFunc(RosMessage msg, TopicManagerAsync.SerializeFunc serfunc, bool ser, bool nc)
        {
            this.msg = msg;
            this.serfunc = serfunc;
            serialize = ser;
            nocopy = nc;
        }
    }
}
