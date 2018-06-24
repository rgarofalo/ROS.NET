using System;
using System.Diagnostics;
using System.Threading.Tasks;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    public class Publisher<M> : IPublisher where M : RosMessage, new()
    {
        private Publication p;

        /// <summary>
        ///     Creates a ros publisher
        /// </summary>
        /// <param name="topic">Topic name to publish to</param>
        /// <param name="md5sum">md5sum for topic and type</param>
        /// <param name="datatype">Datatype to publish</param>
        /// <param name="nodeHandle">nodehandle</param>
        /// <param name="callbacks">Any callbacks to attach</param>
        public Publisher(string topic, string md5sum, string datatype, NodeHandle nodeHandle,
            SubscriberCallbacks callbacks)
        {
            this.topic = topic;
            this.md5sum = md5sum;
            this.datatype = datatype;
            this.nodeHandle = nodeHandle;
            this.callbacks = callbacks;
        }

        public void publish(M msg)
        {
            if (p == null)
                p = TopicManager.Instance.LookupPublication(topic);
            if (p != null)
            {
                msg.Serialized = null;
                TopicManager.Instance.Publish(p, msg);
            }
        }
    }

    public class IPublisher
        : IDisposable
    {
        public SubscriberCallbacks callbacks;

        public string datatype;
        public string md5sum;
        public NodeHandle nodeHandle;
        public string topic;
        public bool unadvertised;

        public bool IsValid
        {
            get { return !unadvertised; }
        }

        internal async Task Unadvertise()
        {
            if (!unadvertised)
            {
                unadvertised = true;
                await TopicManager.Instance.Unadvertise(topic, callbacks);
            }
        }

        public void Dispose()
        {
            var t = Unadvertise();
            t.WhenCompleted().Wait();
        }
    }
}
