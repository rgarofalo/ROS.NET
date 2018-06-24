using System;

namespace Uml.Robotics.Ros
{
    internal abstract class SubscriberLink
        : IDisposable
    {
        public class SubscriberStats
        {
            public long BytesSent;
            public long MessageDataSent;
            public long MessagesSent;
        }

        public int connectionId;
        public string DestinationCallerId = "";
        protected Publication parent;
        public SubscriberStats Stats { get; private set; } = new SubscriberStats();
        public string topic = "";

        public string Md5sum
        {
            get
            {
                lock (parent)
                {
                    return parent.Md5Sum;
                }
            }
        }

        public string DataType
        {
            get
            {
                lock (parent)
                {
                    return parent.DataType;
                }
            }
        }

        public string MessageDefinition
        {
            get
            {
                lock (parent)
                {
                    return parent.MessageDefinition;
                }
            }
        }

        public abstract void EnqueueMessage(MessageAndSerializerFunc holder);
        public abstract void GetPublishTypes(ref bool ser, ref bool nocopy, string type_info);
        public abstract void Dispose();
    }
}
