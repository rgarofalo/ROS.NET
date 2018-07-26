using System;
using System.Collections.Generic;

namespace Uml.Robotics.Ros
{
    internal class LocalPublisherLink : PublisherLink
    {
        private readonly object gate = new object();
        private bool disposed;
        private LocalSubscriberLink subscriberLink;

        public LocalPublisherLink(Subscription parent, string xmlrpc_uri)
            : base(parent, xmlrpc_uri)
        {
        }

        public override string TransportType =>
            "INTRAPROCESS";

        public override bool IsConnected =>
            !disposed;

        public void SetPublisher(LocalSubscriberLink link)
        {
            lock (gate)
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
                subscriberLink = link;
            }
        }

        public override void Dispose()
        {
            lock (gate)
            {
                if (disposed)
                    return;
                disposed = true;
            }

            Parent.RemovePublisherLink(this);
        }

        public void HandleMessage<T>(T m, bool ser, bool nocopy) where T : RosMessage, new()
        {
            lock (gate)
            {
                Stats.MessagesReceived++;

                if (m.Serialized != null)
                {
                    Stats.BytesReceived += m.Serialized.Length;
                }

                Stats.Drops += Parent.HandleMessage(m, ser, nocopy, m.connection_header, this);
            }
        }

        public void GetPublishTypes(ref bool ser, ref bool nocopy, string messageType)
        {
            lock (gate)
            {
                if (disposed)
                {
                    ser = false;
                    nocopy = false;
                    return;
                }
            }

            if (Parent != null)
            {
                Parent.GetPublishTypes(ref ser, ref nocopy, messageType);
            }
            else
            {
                ser = true;
                nocopy = false;
            }
        }
    }
}