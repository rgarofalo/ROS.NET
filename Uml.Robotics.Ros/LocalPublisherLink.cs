using System;
using System.Collections.Generic;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class LocalPublisherLink : PublisherLink
    {
        private object gate = new object();
        private bool dropped;
        private LocalSubscriberLink publisher;

        public LocalPublisherLink(SubscriptionAsync parent, string xmlrpc_uri)
            : base(parent, xmlrpc_uri)
        {
        }

        public override string TransportType
        {
            get { return "INTRAPROCESS"; }
        }

        public void SetPublisher(LocalSubscriberLink pub_link)
        {
            lock (Parent)
            {
                var headerFields = new Dictionary<string, string>
                {
                    ["topic"] = Parent.Name,
                    ["md5sum"] = Parent.Md5Sum,
                    ["callerid"] = ThisNode.Name,
                    ["type"] = Parent.DataType,
                    ["tcp_nodelay"] = "1"
                };
                SetHeader(new Header(headerFields));
            }
        }

        public override void Dispose()
        {
            lock (gate)
            {
                if (dropped)
                    return;
                dropped = true;
            }

            if (publisher != null)
            {
                publisher.Drop();
            }

            lock (Parent)
            {
                Parent.RemovePublisherLink(this);
            }
        }

        public void HandleMessage<T>(T m, bool ser, bool nocopy) where T : RosMessage, new()
        {
            Stats.MessagesReceived++;
            if (m.Serialized == null)
            {
                // ignore stats to avoid an unnecessary allocation
            }
            else
            {
                Stats.BytesReceived += m.Serialized.Length;
            }
            if (Parent != null)
            {
                lock (Parent)
                {
                    Stats.Drops += Parent.HandleMessage(m, ser, nocopy, m.connection_header, this);
                }
            }
        }

        public void GetPublishTypes(ref bool ser, ref bool nocopy, string messageType)
        {
            lock (gate)
            {
                if (dropped)
                {
                    ser = false;
                    nocopy = false;
                    return;
                }
            }
            if (Parent != null)
            {
                lock (Parent)
                {
                    Parent.GetPublishTypes(ref ser, ref nocopy, messageType);
                }
            }
            else
            {
                ser = true;
                nocopy = false;
            }
        }
    }
}