using System;

namespace Uml.Robotics.Ros
{
    public class HeaderErrorException
        : RosException
    {
        public HeaderErrorException(string message)
            : base(message)
        {
        }
    }

    internal abstract class PublisherLink
        : IDisposable
    {
        public class PublisherStats
        {
            public long BytesReceived;
            public long Drops;
            public long MessagesReceived;
        }

        private Header header;

        public int ConnectionId;
        public bool Latched;
        public string Md5Sum = "";

        public PublisherLink(Subscription parent, string xmlrpcUri)
        {
            this.Parent = parent;
            this.XmlRpcUri = xmlrpcUri;
        }

        public string CallerId { get; private set; }
        public Subscription Parent { get; }
        public string XmlRpcUri { get; }
        public PublisherStats Stats { get; } = new PublisherStats();

        public virtual string TransportType =>
            "TCPROS";

        public Header Header =>
            header;

        public void SetHeader(Header header)
        {
            if (!header.Values.ContainsKey("md5sum"))
            {
                throw new HeaderErrorException("Field 'md5sum' missing in connection header.");
            }

            if (!header.Values.ContainsKey("latching"))
            {
                throw new HeaderErrorException("Field 'latching' missing in connection header.");
            }

            this.CallerId = header.Values["callerid"];
            this.Md5Sum = header.Values["md5sum"];
            this.Latched = header.Values["latching"] == "1";
            this.ConnectionId = ConnectionManager.Instance.GetNewConnectionId();
            this.header = header;

            Parent.HandleHeader(this, this.header);
        }

        public abstract void Dispose();
    }
}
