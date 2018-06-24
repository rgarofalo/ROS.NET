using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Uml.Robotics.XmlRpc;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    internal class Subscription
        : IDisposable
    {
        private class CallbackInfo
        {
            public ICallbackQueue Callback;
            public ISubscriptionCallbackHelper Helper;
            public CallbackInterface SubscriptionQueue;
        }

        private class LatchInfo
        {
            public IDictionary<string, string> ConnectionHeader;
            public PublisherLink Link;
            public RosMessage Message;
            public TimeData ReceiptTime;
        }

        private readonly object gate = new object();
        private readonly ILogger logger = ApplicationLogging.CreateLogger<Subscription>();
        private List<PublisherLink> publisherLinks = new List<PublisherLink>();
        private List<PendingConnection> pendingConnections = new List<PendingConnection>();
        private List<CallbackInfo> callbacks = new List<CallbackInfo>();

        private Dictionary<PublisherLink, LatchInfo> latchedMessages = new Dictionary<PublisherLink, LatchInfo>();

        public Subscription(string name, string md5sum, string dataType)
        {
            this.Name = name;
            this.Md5Sum = md5sum;
            this.DataType = dataType;
        }

        public void Dispose()
        {
            if (!IsDisposed)
            {
                IsDisposed = true;
                DropAllConnections();
            }
        }

        public string Name { get; }
        public string Md5Sum { get; private set; }
        public string DataType { get; }
        public bool IsDisposed { get; private set; }

        public int NumPublishers
        {
            get
            {
                lock (gate)
                {
                    return publisherLinks.Count;
                }
            }
        }

        public int NumCallbacks
        {
            get
            {
                lock (gate)
                {
                    return callbacks.Count;
                }
            }
        }

        public XmlRpcValue GetStats()
        {
            var stats = new XmlRpcValue();
            stats.Set(0, Name);
            var conn_data = new XmlRpcValue();
            conn_data.SetArray(0);
            lock (gate)
            {
                int cidx = 0;
                foreach (PublisherLink link in publisherLinks)
                {
                    XmlRpcValue v = new XmlRpcValue();
                    var s = link.Stats;
                    v.Set(0, link.ConnectionId);
                    v.Set(1, s.BytesReceived);
                    v.Set(2, s.MessagesReceived);
                    v.Set(3, s.Drops);
                    v.Set(4, 0);
                    conn_data.Set(cidx++, v);
                }
            }
            stats.Set(1, conn_data);
            return stats;
        }

        public void GetInfo(XmlRpcValue info)
        {
            lock (gate)
            {
                //Logger.LogDebug("SUB: getInfo with " + publisher_links.Count + " publinks in list");
                foreach (PublisherLink c in publisherLinks)
                {
                    //Logger.LogDebug("PUB: adding a curr_info to info!");
                    var curr_info = new XmlRpcValue();
                    curr_info.Set(0, (int)c.ConnectionId);
                    curr_info.Set(1, c.XmlRpcUri);
                    curr_info.Set(2, "i");
                    curr_info.Set(3, c.TransportType);
                    curr_info.Set(4, Name);
                    //Logger.LogDebug("PUB curr_info DUMP:\n\t");
                    //curr_info.Dump();
                    info.Set(info.Count, curr_info);
                }
                //Logger.LogDebug("SUB: outgoing info is of type: " + info.Type + " and has size: " + info.Size);
            }
        }

        public void DropAllConnections()
        {
            List<PublisherLink> subscribers;

            lock (gate)
            {
                subscribers = publisherLinks;
                publisherLinks = new List<PublisherLink>();
            }

            foreach (PublisherLink it in subscribers)
            {
                it.Dispose();
            }
        }

        private static bool UrisEqual(string uri1, string uri2)
        {
            if (uri1 == null)
            {
                throw new ArgumentNullException(nameof(uri1));
            }

            if (uri2 == null)
            {
                throw new ArgumentNullException(nameof(uri2));
            }

            string h1, h2;
            int p1, p2;
            return Network.SplitUri(uri1, out h1, out p1) && Network.SplitUri(uri2, out h2, out p2) && h1 == h2 && p1 == p2;
        }

        public void RemovePublisherLink(PublisherLink pub)
        {
            lock (gate)
            {
                if (publisherLinks.Contains(pub))
                {
                    publisherLinks.Remove(pub);
                }

                if (pub.Latched)
                    latchedMessages.Remove(pub);
            }
        }

        public void AddPublisherLink(PublisherLink pub)
        {
            lock (gate)
            {
                publisherLinks.Add(pub);
            }
        }

        public async Task<bool> PubUpdate(IEnumerable<string> publisherUris)
        {
            using (logger.BeginScope(nameof(PubUpdate)))
            {
                lock (gate)
                {
                    if (IsDisposed)
                        return false;
                }

                bool retval = true;

                logger.LogDebug($"Publisher update for [{Name}]");

                var additions = new List<string>();
                List<PublisherLink> subtractions;
                lock (gate)
                {
                    subtractions = publisherLinks.Where(x => !publisherUris.Any(u => UrisEqual(x.XmlRpcUri, u))).ToList();
                    foreach (string uri in publisherUris)
                    {
                        bool found = publisherLinks.Any(spc => UrisEqual(uri, spc.XmlRpcUri));
                        if (found)
                            continue;

                        lock (pendingConnections)
                        {
                            if (pendingConnections.Any(pc => UrisEqual(uri, pc.RemoteUri)))
                            {
                                found = true;
                            }

                            if (!found)
                            {
                                additions.Add(uri);
                            }
                        }
                    }
                }

                foreach (PublisherLink link in subtractions)
                {
                    if (link.XmlRpcUri != XmlRpcManager.Instance.Uri)
                    {
                        logger.LogDebug($"Disconnecting from publisher [{link.CallerId}] of topic [{Name}] at [{link.XmlRpcUri}]");
                        link.Dispose();
                    }
                    else
                    {
                        logger.LogWarning("Cannot disconnect from self for topic: " + Name);
                    }
                }

                foreach (string i in additions)
                {
                    if (XmlRpcManager.Instance.Uri != i)
                    {
                        retval &= await NegotiateConnection(i);
                    }
                }
                return retval;
            }
        }

        private async Task<bool> NegotiateConnection(string xmlRpcUri)
        {
            int protos = 0;
            XmlRpcValue tcprosArray = new XmlRpcValue(), protosArray = new XmlRpcValue(), Params = new XmlRpcValue();
            tcprosArray.Set(0, "TCPROS");
            protosArray.Set(protos++, tcprosArray);
            Params.Set(0, ThisNode.Name);
            Params.Set(1, Name);
            Params.Set(2, protosArray);

            if (!Network.SplitUri(xmlRpcUri, out string peerHost, out int peerPort))
            {
                logger.LogError($"Bad XML-RPC URI: [{xmlRpcUri}]");
                return false;
            }

            var client = new XmlRpcClient(peerHost, peerPort);
            var requestTopicTask = client.ExecuteAsync("requestTopic", Params);

            logger.LogDebug($"Began asynchronous XML RPC connection to http://{peerHost}:{peerPort}/ for topic [{Name}]");

            var conn = new PendingConnection(client, requestTopicTask, xmlRpcUri);

            lock (pendingConnections)
            {
                pendingConnections.Add(conn);
            }

            await requestTopicTask.WhenCompleted();

            PendingConnectionDone(conn, requestTopicTask);

            return requestTopicTask.IsCompletedSuccessfully;
        }

        private void PendingConnectionDone(PendingConnection conn, Task<XmlRpcCallResult> callTask)
        {
            lock (pendingConnections)
            {
                pendingConnections.Remove(conn);
            }

            using (logger.BeginScope(nameof(PendingConnectionDone)))
            {
                if (callTask.IsFaulted)
                {
                    logger.LogWarning($"Negotiating for {Name} has failed (Error: {callTask.Exception.Message}).");
                    return;
                }

                if (!callTask.Result.Success)
                {
                    logger.LogWarning($"Negotiating for {Name} has failed. XML-RCP call failed.");
                    return;
                }

                var resultValue = callTask.Result.Value;

                lock (gate)
                {
                    if (IsDisposed)
                        return;
                }

                var proto = new XmlRpcValue();
                if (!XmlRpcManager.Instance.ValidateXmlRpcResponse("requestTopic", resultValue, proto))
                {
                    logger.LogWarning($"Negotiating for {Name} has failed.");
                    return;
                }

                string peerHost = conn.Client.Host;
                int peerPort = conn.Client.Port;
                string xmlrpcUri = "http://" + peerHost + ":" + peerPort + "/";
                if (proto.Count == 0)
                {
                    logger.LogDebug(
                        $"Could not agree on any common protocols with [{xmlrpcUri}] for topic [{Name}]"
                    );
                    return;
                }
                if (proto.Type != XmlRpcType.Array)
                {
                    logger.LogWarning($"Available protocol info returned from {xmlrpcUri} is not a list.");
                    return;
                }

                string protoName = proto[0].GetString();
                if (protoName == "UDPROS")
                {
                    logger.LogError("UDP is currently not supported. Use TCPROS instead.");
                }
                else if (protoName == "TCPROS")
                {
                    if (proto.Count != 3 || proto[1].Type != XmlRpcType.String || proto[2].Type != XmlRpcType.Int)
                    {
                        logger.LogWarning("TcpRos Publisher should implement string, int as parameter");
                        return;
                    }

                    string pubHost = proto[1].GetString();
                    int pubPort = proto[2].GetInt();
                    logger.LogDebug($"Connecting via tcpros to topic [{Name}] at host [{pubHost}:{pubPort}]");

                    try
                    {
                        var pubLink = new TransportPublisherLink(this, xmlrpcUri);
                        lock (gate)
                        {
                            pubLink.Initialize(pubHost, pubPort);
                            AddPublisherLink(pubLink);
                        }

                        logger.LogDebug($"Connected to publisher of topic [{Name}] at  [{pubHost}:{pubPort}]");
                    }
                    catch
                    {
                        logger.LogError($"Failed to connect to publisher of topic [{Name}] at [{pubHost}:{pubPort}]");
                    }
                }
                else
                {
                    logger.LogError("The XmlRpc Server does not provide a supported protocol.");
                }
            }
        }

        internal void HandleHeader(PublisherLink link, Header header)
        {
            lock (gate)
            {
                if (Md5Sum == "*")
                    Md5Sum = link.Md5Sum;
            }
        }

        internal long HandleMessage(
            RosMessage msg,
            bool ser,
            bool nocopy,
            IDictionary<string, string> connectionHeader,
            PublisherLink link
        )
        {
            RosMessage t = null;
            long drops = 0;
            TimeData receipt_time = ROS.GetTime().data;
            if (msg.Serialized != null) // will be null if self-subscribed
                msg.Deserialize(msg.Serialized);

            lock (gate)
            {
                foreach (CallbackInfo info in callbacks)
                {
                    string ti = info.Helper.type;
                    if (nocopy || ser)
                    {
                        t = msg;
                        t.connection_header = msg.connection_header;
                        t.Serialized = null;
                        bool was_full = false;
                        bool nonconst_need_copy = callbacks.Count > 1;
                        info.SubscriptionQueue.AddToCallbackQueue(info.Helper, t, nonconst_need_copy, ref was_full, receipt_time);
                        if (was_full)
                            ++drops;
                        else
                            info.Callback.AddCallback(info.SubscriptionQueue);
                    }
                }
            }

            if (t != null && link.Latched)
            {
                LatchInfo li = new LatchInfo
                {
                    Message = t,
                    Link = link,
                    ConnectionHeader = connectionHeader,
                    ReceiptTime = receipt_time
                };
                if (latchedMessages.ContainsKey(link))
                    latchedMessages[link] = li;
                else
                    latchedMessages.Add(link, li);
            }

            return drops;
        }

        internal bool AddCallback(
            ISubscriptionCallbackHelper helper,
            string md5sum,
            ICallbackQueue queue,
            int queueSize,
            bool allowConcurrentCallbacks,
            string topic
        )
        {
            lock (gate)
            {
                if (this.Md5Sum == "*" && md5sum != "*")
                    this.Md5Sum = md5sum;

                if (md5sum != "*" && md5sum != this.Md5Sum)
                    return false;

                var info = new CallbackInfo
                {
                    Helper = helper,
                    Callback = queue,
                    SubscriptionQueue = new Callback(helper.Callback.SendEvent, topic, queueSize, allowConcurrentCallbacks)
                };

                callbacks.Add(info);

                if (latchedMessages.Count > 0)
                {
                    string ti = info.Helper.type;
                    var receiptTime = ROS.GetTime().data;
                    foreach (PublisherLink link in publisherLinks)
                    {
                        if (link.Latched)
                        {
                            if (latchedMessages.ContainsKey(link))
                            {
                                LatchInfo latch_info = latchedMessages[link];
                                bool wasFull = false;
                                bool nonconst_need_copy = callbacks.Count > 1;
                                info.SubscriptionQueue.AddToCallbackQueue(info.Helper, latchedMessages[link].Message, nonconst_need_copy, ref wasFull, receiptTime);
                                if (!wasFull)
                                {
                                    info.Callback.AddCallback(info.SubscriptionQueue);
                                }
                            }
                        }
                    }
                }

                return true;
            }
        }

        public void RemoveCallback(ISubscriptionCallbackHelper helper)
        {
            lock (gate)
            {
                foreach (CallbackInfo info in callbacks)
                {
                    if (info.Helper == helper)
                    {
                        info.SubscriptionQueue.Clear();
                        info.Callback.RemoveById(info.SubscriptionQueue.Uid);
                        callbacks.Remove(info);

                        break;
                    }
                }
            }
        }

        public void AddLocalConnection(Publication pub)
        {
            lock (gate)
            {
                if (IsDisposed)
                    return;

                logger.LogInformation("Creating intraprocess link for topic [{0}]", Name);

                var pub_link = new LocalPublisherLink(this, XmlRpcManager.Instance.Uri);
                var sub_link = new LocalSubscriberLink(pub);
                pub_link.SetPublisher(sub_link);
                sub_link.SetSubscriber(pub_link);

                AddPublisherLink(pub_link);
                pub.AddSubscriberLink(sub_link);
            }
        }

        public void GetPublishTypes(ref bool ser, ref bool nocopy, string typeInfo)
        {
            lock (gate)
            {
                foreach (CallbackInfo info in callbacks)
                {
                    if (info.Helper.type == typeInfo)
                        nocopy = true;
                    else
                        ser = true;
                    if (nocopy && ser)
                        return;
                }
            }
        }
    }
}
