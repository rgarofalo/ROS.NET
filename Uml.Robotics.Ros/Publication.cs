using System;
using System.Collections.Generic;
using System.Reflection;
using System.Linq;
using Microsoft.Extensions.Logging;

using Uml.Robotics.XmlRpc;
using std_msgs = Messages.std_msgs;
using Xamla.Robotics.Ros.Async;
using System.Threading.Tasks;
using System.Threading;

namespace Uml.Robotics.Ros
{
    public class Publication
        : IDisposable
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<Publication>();
        private readonly object gate = new object();
        private uint _seq;
        private List<SubscriberLink> subscriberLinks = new List<SubscriberLink>();
        private List<SubscriberCallbacks> callbacks = new List<SubscriberCallbacks>();

        internal MessageAndSerializerFunc lastMessage;
        internal AsyncQueue<MessageAndSerializerFunc> publishQueue = new AsyncQueue<MessageAndSerializerFunc>(65535, true);

        CancellationTokenSource cts;
        CancellationToken cancel;
        Task publishLoopTask;

        public bool Dropped;
        public Header connectionHeader;

        public Publication(
            string name,
            string datatype,
            string md5sum,
            string messageDefinition,
            int maxQueue,
            bool latch,
            bool hasHeader
        )
        {
            this.Name = name;
            this.DataType = datatype;
            this.Md5Sum = md5sum;
            this.MessageDefinition = messageDefinition;
            this.MaxQueue = maxQueue;
            this.Latch = latch;
            this.HasHeader = hasHeader;
        }

        public void Dispose()
        {
            lock (gate)
            {
                if (Dropped)
                    return;
                Dropped = true;
            }
            DropAllConnections();
        }

        public string DataType { get; }
        public bool HasHeader { get; }
        public bool Latch { get; }
        public int MaxQueue { get; }
        public string Md5Sum { get; }
        public string MessageDefinition { get; }
        public string Name { get; }

        public int NumCallbacks
        {
            get { lock (gate) return callbacks.Count; }
        }

        public bool HasSubscribers
        {
            get { lock (gate) return subscriberLinks.Count > 0; }
        }

        public int NumSubscribers
        {
            get { lock (gate) return subscriberLinks.Count; }
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
                foreach (SubscriberLink sub_link in subscriberLinks)
                {
                    var s = sub_link.Stats;
                    var inside = new XmlRpcValue();
                    inside.Set(0, sub_link.connectionId);
                    inside.Set(1, s.BytesSent);
                    inside.Set(2, s.MessageDataSent);
                    inside.Set(3, s.MessagesSent);
                    inside.Set(4, 0);
                    conn_data.Set(cidx++, inside);
                }
            }

            stats.Set(1, conn_data);
            return stats;
        }

        internal void AddSubscriberLink(SubscriberLink link)
        {
            lock (gate)
            {
                if (Dropped)
                    return;

                subscriberLinks.Add(link);
                this.StartPublishLoop();
            }

            if (Latch && lastMessage != null)
            {
                link.EnqueueMessage(lastMessage);
            }

            HandlePeerConnect(link);
        }

        internal void RemoveSubscriberLink(SubscriberLink link)
        {
            SubscriberLink lnk = null;
            lock (gate)
            {
                if (Dropped)
                    return;
                if (subscriberLinks.Contains(link))
                {
                    lnk = link;
                    subscriberLinks.Remove(lnk);
                    if (subscriberLinks.Count == 0)
                    {
                        StopPublishLoop();
                    }
                }
            }
            if (lnk != null)
                HandlePeerDisconnect(lnk);
        }

        internal void Publish(MessageAndSerializerFunc msg)
        {
            lock (gate)
            {
                publishQueue.TryOnNext(msg);
            }
        }

        private void StartPublishLoop()
        {
            lock (gate)
            {
                if (publishLoopTask != null && !publishLoopTask.IsCompleted || publishQueue.IsCompleted)
                    return;

                cts = new CancellationTokenSource();
                cancel = cts.Token;
                publishLoopTask = RunPublishLoopAsync();
            }
        }

        private void StopPublishLoop()
        {
            Task publishTask = null;

            lock (gate)
            {
                if (cts != null && publishLoopTask != null)
                {
                    cts.Cancel();
                    publishTask = publishLoopTask;
                }
            }

            publishTask?.WhenCompleted()?.Wait();
        }

        private async Task RunPublishLoopAsync()
        {
            while (await publishQueue.MoveNext(cancel))
            {
                cancel.ThrowIfCancellationRequested();

                if (Dropped)
                    return;

                EnqueueMessage(publishQueue.Current);
            }
        }

        public bool ValidateHeader(Header header, out string errorMessage)
        {
            string md5sum = "", topic = "", client_callerid = "";
            if (!header.Values.ContainsKey("md5sum") || !header.Values.ContainsKey("topic") ||
                !header.Values.ContainsKey("callerid"))
            {
                const string msg = "Header from subscriber did not have the required elements: md5sum, topic, callerid";
                logger.LogWarning(msg);
                errorMessage = msg;
                return false;
            }
            md5sum = (string) header.Values["md5sum"];
            topic = (string) header.Values["topic"];
            client_callerid = (string) header.Values["callerid"];
            if (Dropped)
            {
                string msg = "Received a tcpros connection for a nonexistent topic [" + topic + "] from [" +
                             client_callerid + "].";
                logger.LogWarning(msg);
                errorMessage = msg;
                return false;
            }

            if (Md5Sum != md5sum && (md5sum != "*") && Md5Sum != "*")
            {
                string datatype = header.Values.ContainsKey("type") ? (string) header.Values["type"] : "unknown";
                string msg = "Client [" + client_callerid + "] wants topic [" + topic + "] to hava datatype/md5sum [" +
                             datatype + "/" + md5sum + "], but our version has [" + DataType + "/" + Md5Sum +
                             "]. Dropping connection";
                logger.LogWarning(msg);
                errorMessage = msg;
                return false;
            }

            errorMessage = null;
            return true;
        }

        public void GetInfo(XmlRpcValue info)
        {
            lock (gate)
            {
                foreach (SubscriberLink c in subscriberLinks)
                {
                    var curr_info = new XmlRpcValue();
                    curr_info.Set(0, (int) c.connectionId);
                    curr_info.Set(1, c.DestinationCallerId);
                    curr_info.Set(2, "o");
                    curr_info.Set(3, "TCPROS");
                    curr_info.Set(4, Name);
                    info.Set(info.Count, curr_info);
                }
            }
        }

        public void AddCallbacks(SubscriberCallbacks callbacks)
        {
            lock (gate)
            {
                this.callbacks.Add(callbacks);
                if (callbacks.OnConnect != null && callbacks.CallbackQueue != null)
                {
                    foreach (SubscriberLink i in subscriberLinks)
                    {
                        CallbackInterface cb = new PeerConnDisconnCallback(callbacks.OnConnect, i);
                        callbacks.CallbackId = cb.Uid;
                        callbacks.CallbackQueue.AddCallback(cb);
                    }
                }
            }
        }

        public void RemoveCallbacks(SubscriberCallbacks callbacks)
        {
            lock (gate)
            {
                if (callbacks.CallbackId >= 0)
                    callbacks.CallbackQueue.RemoveById(callbacks.CallbackId);
                if (this.callbacks.Contains(callbacks))
                    this.callbacks.Remove(callbacks);
            }
        }

        internal bool EnqueueMessage(MessageAndSerializerFunc holder)
        {
            lock (gate)
            {
                if (Dropped)
                    return false;
            }

            uint seq = IncrementSequence();

            if (HasHeader)
            {
                object h = holder.msg.GetType().GetTypeInfo().GetField("header").GetValue(holder.msg);

                std_msgs.Header header;
                if (h == null)
                    header = new std_msgs.Header();
                else
                    header = (std_msgs.Header) h;

                header.seq = seq;
                if (header.stamp == null)
                {
                    header.stamp = ROS.GetTime();
                }
                if (header.frame_id == null)
                {
                    header.frame_id = "";
                }
                holder.msg.GetType().GetTypeInfo().GetField("header").SetValue(holder.msg, header);
            }
            holder.msg.connection_header = connectionHeader.Values;

            lock (gate)
            {
                foreach (SubscriberLink link in subscriberLinks)
                {
                    link.EnqueueMessage(holder);
                }
            }

            if (Latch)
            {
                lastMessage = new MessageAndSerializerFunc(holder.msg, holder.serfunc, false, true);
            }

            return true;
        }

        public void DropAllConnections()
        {
            List<SubscriberLink> subscribers = null;
            lock (gate)
            {
                subscribers = new List<SubscriberLink>(subscriberLinks);
                subscriberLinks.Clear();
            }

            foreach (SubscriberLink link in subscribers)
            {
                link.Dispose();
            }

            subscribers.Clear();
        }

        internal void HandlePeerConnect(SubscriberLink sub_link)
        {
            //Logger.LogDebug($"PEER CONNECT: Id: {sub_link.connection_id} Dest: {sub_link.destination_caller_id} Topic: {sub_link.topic}");
            foreach (SubscriberCallbacks cbs in callbacks)
            {
                if (cbs.OnConnect != null && cbs.CallbackQueue != null)
                {
                    var cb = new PeerConnDisconnCallback(cbs.OnConnect, sub_link);
                    cbs.CallbackId = cb.Uid;
                    cbs.CallbackQueue.AddCallback(cb);
                }
            }
        }

        internal void HandlePeerDisconnect(SubscriberLink sub_link)
        {
            //Logger.LogDebug("PEER DISCONNECT: [" + sub_link.topic + "]");
            foreach (SubscriberCallbacks cbs in callbacks)
            {
                if (cbs.OnDisconnect != null && cbs.CallbackQueue != null)
                {
                    var cb = new PeerConnDisconnCallback(cbs.OnDisconnect, sub_link);
                    cbs.CallbackId = cb.Uid;
                    cbs.CallbackQueue.AddCallback(cb);
                }
            }
        }

        public uint IncrementSequence()
        {
            lock (gate)
            {
                return _seq++;
            }
        }

        internal void GetPublishTypes(ref bool serialize, ref bool nocopy, string messageType)
        {
            lock (gate)
            {
                foreach (SubscriberLink sub in subscriberLinks)
                {
                    bool s = false, n = false;
                    sub.GetPublishTypes(ref s, ref n, messageType);
                    serialize = serialize || s;
                    nocopy = nocopy || n;
                    if (serialize && nocopy)
                        break;
                }
            }
        }
    }

    internal class PeerConnDisconnCallback : CallbackInterface
    {
        private readonly ILogger logger = ApplicationLogging.CreateLogger<PeerConnDisconnCallback>();
        private readonly SubscriberStatusCallback callback;
        private readonly SubscriberLink subscriberLink;

        public PeerConnDisconnCallback(SubscriberStatusCallback callback, SubscriberLink subscriberLink)
        {
            this.callback = callback;
            this.subscriberLink = subscriberLink;
        }

        internal override CallResult Call()
        {
            ROS.Debug()("Called PeerConnDisconnCallback");
            SingleSubscriberPublisher pub = new SingleSubscriberPublisher(subscriberLink);
            logger.LogDebug($"Callback: Name: {pub.SubscriberName} Topic: {pub.Topic}");
            callback(pub);
            return CallResult.Success;
        }

        public override void AddToCallbackQueue(ISubscriptionCallbackHelper helper, RosMessage msg, bool nonconst_need_copy, ref bool wasFull, TimeData receiptTime)
        {
            throw new NotImplementedException();
        }

        public override void Clear()
        {
            throw new NotImplementedException();
        }
    }
}
