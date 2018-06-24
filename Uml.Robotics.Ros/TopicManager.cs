using Microsoft.Extensions.Logging;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Uml.Robotics.XmlRpc;
using Xamla.Robotics.Ros.Async;

namespace Uml.Robotics.Ros
{
    public class SubscribeFailedException : RosException
    {
        public SubscribeFailedException(SubscribeOptions ops, string reason)
            : base($"Subscribing to topic [{ops.topic}] failed: {reason}")
        {
        }
    }

    public class TopicManager
    {
        public delegate byte[] SerializeFunc();

        private static Lazy<TopicManager> instance = new Lazy<TopicManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        public static TopicManager Instance =>
            instance.Value;

        internal static void Terminate() =>
            Instance.Shutdown().Wait();

        internal static void Reset() =>
            instance = new Lazy<TopicManager>(LazyThreadSafetyMode.ExecutionAndPublication);

        private readonly ILogger logger = ApplicationLogging.CreateLogger<TopicManager>();
        private object gate = new object();
        private bool shuttingDown;
        private List<Publication> advertisedTopics = new List<Publication>();
        private List<Subscription> subscriptions = new List<Subscription>();

        /// <summary>
        /// Binds the XmlRpc requests to callback functions, signal to start
        /// </summary>
        public void Start()
        {
            lock (gate)
            {
                shuttingDown = false;

                XmlRpcManager.Instance.Bind("publisherUpdate", PublisherUpdateCallback);
                XmlRpcManager.Instance.Bind("requestTopic", RequestTopicCallback);
                XmlRpcManager.Instance.Bind("getBusStats", GetBusStatsCallback);
                XmlRpcManager.Instance.Bind("getBusInfo", GetBusInfoCallback);
                XmlRpcManager.Instance.Bind("getSubscriptions", GetSubscriptionsCallback);
                XmlRpcManager.Instance.Bind("getPublications", GetPublicationsCallback);
            }
        }

        /// <summary>
        /// Unbinds the XmlRpc requests to callback functions, signal to shutdown
        /// </summary>
        public async Task Shutdown()
        {
            List<Publication> pubs;
            List<Subscription> subs;

            lock (gate)
            {
                if (shuttingDown)
                    return;

                shuttingDown = true;

                XmlRpcManager.Instance.Unbind("publisherUpdate");
                XmlRpcManager.Instance.Unbind("requestTopic");
                XmlRpcManager.Instance.Unbind("getBusStats");
                XmlRpcManager.Instance.Unbind("getBusInfo");
                XmlRpcManager.Instance.Unbind("getSubscriptions");
                XmlRpcManager.Instance.Unbind("getPublications");

                pubs = advertisedTopics.ToList();
                advertisedTopics.Clear();
                subs = subscriptions.ToList();
                subscriptions.Clear();
            }

            bool failedOnceToUnadvertise = false;
            foreach (Publication p in pubs)
            {
                if (!p.Dropped && !failedOnceToUnadvertise)
                {
                    try
                    {
                        failedOnceToUnadvertise = !await UnregisterPublisher(p.Name);
                    }
                    catch
                    {
                        failedOnceToUnadvertise = true;
                    }
                }
                p.Dispose();
            }

            bool failedOnceToUnsubscribe = false;
            foreach (Subscription s in subs)
            {
                if (!s.IsDisposed && !failedOnceToUnsubscribe)
                {
                    try
                    {
                        failedOnceToUnsubscribe = !await UnregisterSubscriber(s.Name);
                    }
                    catch
                    {
                        failedOnceToUnsubscribe = true;
                    }
                }
                s.Dispose();
            }
        }

        /// <summary>
        ///  Gets the list of advertised topics.
        /// </summary>
        /// <returns>List of topics</returns>
        public string[] GetAdvertisedTopics()
        {
            lock (gate)
            {
                return advertisedTopics.Select(a => a.Name).ToArray();
            }
        }

        /// <summary>
        /// Gets the list of subscribed topics.
        /// </summary>
        public string[] GetSubscribedTopics()
        {
            lock (gate)
            {
                return subscriptions.Select(s => s.Name).ToArray();
            }
        }

        /// <summary>
        /// Looks up all current publishers on a given topic
        /// </summary>
        /// <param name="topic">Topic name to look up</param>
        /// <returns></returns>
        public Publication LookupPublication(string topic)
        {
            lock (gate)
            {
                return LookupPublicationWithoutLock(topic);
            }
        }

        /// <summary>
        ///     Checks if the given topic is valid.
        /// </summary>
        /// <typeparam name="T">Advertise Options </typeparam>
        /// <param name="ops"></param>
        /// <returns></returns>
        private bool IsValid<T>(AdvertiseOptions<T> ops) where T : RosMessage, new()
        {
            if (ops.dataType == "*")
                throw new Exception("Advertising with * as the datatype is not allowed.  Topic [" + ops.topic + "]");
            if (ops.md5Sum == "*")
                throw new Exception("Advertising with * as the md5sum is not allowed.  Topic [" + ops.topic + "]");
            if (ops.md5Sum == "")
                throw new Exception("Advertising on topic [" + ops.topic + "] with an empty md5sum");
            if (ops.dataType == "")
                throw new Exception("Advertising on topic [" + ops.topic + "] with an empty datatype");
            if (string.IsNullOrEmpty(ops.messageDefinition))
            {
                this.logger.LogWarning(
                    "Advertising on topic [" + ops.topic +
                     "] with an empty message definition. Some tools may not work correctly"
                );
            }
            return true;
        }

        /// <summary>
        /// Register as a publisher on a topic.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <param name="ops"></param>
        /// <param name="callbacks"></param>
        /// <returns></returns>
        public async Task<bool> Advertise<T>(AdvertiseOptions<T> ops, SubscriberCallbacks callbacks) where T : RosMessage, new()
        {
            if (!IsValid(ops))
                return false;

            Publication pub = null;
            lock (gate)
            {
                if (shuttingDown)
                    return false;

                pub = LookupPublicationWithoutLock(ops.topic);
                if (pub != null)
                {
                    if (pub.Md5Sum != ops.md5Sum)
                    {
                        this.logger.LogError(
                            "Tried to advertise on topic [{0}] with md5sum [{1}] and datatype [{2}], but the topic is already advertised as md5sum [{3}] and datatype [{4}]",
                            ops.topic,
                            ops.md5Sum,
                            ops.dataType,
                            pub.Md5Sum,
                            pub.DataType
                        );
                        return false;
                    }
                }
                else
                {
                    pub = new Publication(
                        ops.topic,
                        ops.dataType,
                        ops.md5Sum,
                        ops.messageDefinition,
                        ops.queueSize,
                        ops.Latch,
                        ops.hasHeader
                    );
                }
                pub.AddCallbacks(callbacks);
                advertisedTopics.Add(pub);
            }

            bool found = false;
            Subscription sub = null;
            lock (gate)
            {
                foreach (Subscription s in subscriptions)
                {
                    if (s.Name == ops.topic && Md5SumsMatch(s.Md5Sum, ops.md5Sum) && !s.IsDisposed)
                    {
                        found = true;
                        sub = s;
                        break;
                    }
                }
            }

            if (found)
            {
                sub.AddLocalConnection(pub);
            }

            var args = new XmlRpcValue(ThisNode.Name, ops.topic, ops.dataType, XmlRpcManager.Instance.Uri);
            var result = new XmlRpcValue();
            var payload = new XmlRpcValue();

            if (!await Master.ExecuteAsync("registerPublisher", args, result, payload, true))
            {
                this.logger.LogError($"RPC \"registerPublisher\" for topic '{ops.topic}' failed.");
                return false;
            }

            return true;
        }

        public async Task Subscribe(SubscribeOptions ops)
        {
            lock (gate)
            {
                if (AddSubCallback(ops))
                    return;
                if (shuttingDown)
                    return;
            }

            if (string.IsNullOrEmpty(ops.md5sum))
                throw new SubscribeFailedException(ops, "with an empty md5sum");
            if (string.IsNullOrEmpty(ops.datatype))
                throw new SubscribeFailedException(ops, "with an empty datatype");
            if (ops.helper == null)
                throw new SubscribeFailedException(ops, "without a callback");

            string md5sum = ops.md5sum;
            string datatype = ops.datatype;
            var s = new Subscription(ops.topic, md5sum, datatype);
            s.AddCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.allow_concurrent_callbacks, ops.topic);
            if (!await RegisterSubscriber(s, ops.datatype))
            {
                string error = $"Couldn't register subscriber on topic [{ops.topic}]";
                s.Dispose();
                this.logger.LogError(error);
                throw new RosException(error);
            }

            lock (gate)
            {
                subscriptions.Add(s);
            }
        }

        public async Task<bool> Unsubscribe(string topic, ISubscriptionCallbackHelper sbch)
        {
            Subscription sub = null;
            lock (gate)
            {
                if (shuttingDown)
                    return false;

                foreach (Subscription s in subscriptions)
                {
                    if (s.Name == topic)
                    {
                        sub = s;
                        break;
                    }
                }
            }

            if (sub == null)
                return false;

            sub.RemoveCallback(sbch);
            if (sub.NumCallbacks == 0)
            {
                lock (gate)
                {
                    subscriptions.Remove(sub);
                }

                if (!await UnregisterSubscriber(topic))
                    this.logger.LogWarning("Couldn't unregister subscriber for topic [" + topic + "]");

                sub.Dispose();
                return true;
            }
            return true;
        }

        internal Subscription RetSubscription(string topic)
        {
            lock (gate)
            {
                if (!shuttingDown)
                {
                    return subscriptions.FirstOrDefault(s => !s.IsDisposed && s.Name == topic);
                }
            }
            return null;
        }

        public int NumSubscriptions
        {
            get
            {
                lock (gate)
                {
                    return subscriptions.Count;
                }
            }
        }

        internal Subscription GetSubscription(string topic)
        {
            lock (gate)
            {
                if (shuttingDown)
                    return null;

                return subscriptions.FirstOrDefault(t => !t.IsDisposed && t.Name == topic);
            }
        }

        public void Publish(Publication p, RosMessage msg, SerializeFunc serfunc = null)
        {
            if (msg == null)
                return;

            if (serfunc == null)
                serfunc = msg.Serialize;

            if (p.connectionHeader == null)
            {
                var fields = new Dictionary<string, string>()
                {
                    ["type"] = p.DataType,
                    ["md5sum"] = p.Md5Sum,
                    ["message_definition"] = p.MessageDefinition,
                    ["callerid"] = ThisNode.Name,
                    ["latching"] = p.Latch ? "1" : "0"
                };
                p.connectionHeader = new Header(fields);
            }

            if (!ROS.OK || shuttingDown)
                return;

            if (p.HasSubscribers || p.Latch)
            {
                bool nocopy = false;
                bool serialize = false;
                if (msg != null && msg.MessageType != "undefined/unknown")
                {
                    p.GetPublishTypes(ref serialize, ref nocopy, msg.MessageType);
                }
                else
                {
                    serialize = true;
                }

                p.Publish(new MessageAndSerializerFunc(msg, serfunc, serialize, nocopy));
            }
            else
            {
                p.IncrementSequence();
            }
        }

        public void IncrementSequence(string topic)
        {
            Publication pub = LookupPublication(topic);
            pub?.IncrementSequence();
        }

        public bool IsLatched(string topic)
        {
            Publication pub = LookupPublication(topic);
            return pub?.Latch ?? false;
        }

        public bool Md5SumsMatch(string a, string b) =>
            a == "*" || b == "*" || a == b;

        public bool AddSubCallback(SubscribeOptions options)
        {
            bool found = false;
            bool foundTopic = false;
            Subscription sub = null;

            if (shuttingDown)
                return false;

            foreach (Subscription s in subscriptions)
            {
                sub = s;
                if (!sub.IsDisposed && sub.Name == options.topic)
                {
                    foundTopic = true;
                    if (Md5SumsMatch(options.md5sum, sub.Md5Sum))
                        found = true;
                    break;
                }
            }

            if (foundTopic && !found)
            {
                throw new Exception
                    ("Tried to subscribe to a topic with the same name but different md5sum as a topic that was already subscribed [" +
                     options.datatype + "/" + options.md5sum + " vs. " + sub.DataType + "/" +
                     sub.Md5Sum+ "]");
            }

            if (found)
            {
                if (!sub.AddCallback(options.helper, options.md5sum, options.callback_queue, options.queue_size,
                        options.allow_concurrent_callbacks, options.topic))
                {
                    return false;
                }
            }
            return found;
        }

        public bool RequestTopic(string topic, XmlRpcValue protos, ref XmlRpcValue ret)
        {
            foreach (var proto in protos)
            {
                if (proto.Type != XmlRpcType.Array)
                {
                    this.logger.LogError("requestTopic protocol list was not a list of lists");
                    return false;
                }

                if (proto[0].Type != XmlRpcType.String)
                {
                    this.logger.LogError(
                        "requestTopic received a protocol list in which a sublist did not start with a string");
                    return false;
                }

                string proto_name = proto[0].GetString();

                if (proto_name == "TCPROS")
                {
                    var tcpRosParams = new XmlRpcValue("TCPROS", Network.Host, ConnectionManager.Instance.TCPPort);
                    ret.Set(0, 1);
                    ret.Set(1, "");
                    ret.Set(2, tcpRosParams);
                    return true;
                }

                if (proto_name == "UDPROS")
                {
                    this.logger.LogWarning("Ignoring topics with 'UDPROS' as protocol");
                }
                else
                {
                    this.logger.LogWarning("An unsupported protocol was offered: [{0}]", proto_name);
                }
            }

            this.logger.LogError("No supported protocol was provided");
            return false;
        }

        public bool IsTopicAdvertised(string topic)
        {
            return advertisedTopics.Count(o => o.Name == topic) > 0;
        }

        internal async Task<bool> RegisterSubscriber(Subscription s, string datatype)
        {
            string uri = XmlRpcManager.Instance.Uri;

            var args = new XmlRpcValue(ThisNode.Name, s.Name, datatype, uri);
            var result = new XmlRpcValue();
            var payload = new XmlRpcValue();

            if (!await Master.ExecuteAsync("registerSubscriber", args, result, payload, true))
            {
                logger.LogError("RPC \"registerSubscriber\" for service " + s.Name + " failed.");
                return false;
            }

            var pub_uris = new List<string>();
            for (int i = 0; i < payload.Count; i++)
            {
                XmlRpcValue load = payload[i];
                string pubed = load.GetString();
                if (pubed != uri && !pub_uris.Contains(pubed))
                {
                    pub_uris.Add(pubed);
                }
            }

            bool self_subscribed = false;
            Publication pub = null;
            string sub_md5sum = s.Md5Sum;

            lock (gate)
            {
                foreach (Publication p in advertisedTopics)
                {
                    pub = p;
                    string pub_md5sum = pub.Md5Sum;
                    if (pub.Name == s.Name && Md5SumsMatch(pub_md5sum, sub_md5sum) && !pub.Dropped)
                    {
                        self_subscribed = true;
                        break;
                    }
                }
            }

            await s.PubUpdate(pub_uris);
            if (self_subscribed)
            {
                s.AddLocalConnection(pub);
            }

            return true;
        }

        public async Task<bool> UnregisterSubscriber(string topic)
        {
            bool unregisterSuccess = false;
            try
            {
                var args = new XmlRpcValue(ThisNode.Name, topic, XmlRpcManager.Instance.Uri);
                var result = new XmlRpcValue();
                var payload = new XmlRpcValue();
                unregisterSuccess = await Master.ExecuteAsync("unregisterSubscriber", args, result, payload, false) && result.IsEmpty;
            }
            catch
            {
                // ignore exception during unregister
            }
            return unregisterSuccess;
        }

        public async Task<bool> UnregisterPublisher(string topic)
        {
            bool unregisterSuccess = false;
            try
            {
                var args = new XmlRpcValue(ThisNode.Name, topic, XmlRpcManager.Instance.Uri);
                var result = new XmlRpcValue();
                var payload = new XmlRpcValue();
                unregisterSuccess = await Master.ExecuteAsync("unregisterPublisher", args, result, payload, false) && result.IsEmpty;
            }
            catch
            {
                // ignore exception during unregister
            }
            return unregisterSuccess;
        }

        private Publication LookupPublicationWithoutLock(string topic)
        {
            return advertisedTopics.FirstOrDefault(p => p.Name == topic && !p.Dropped);
        }

        public XmlRpcValue GetBusStats()
        {
            var publishStats = new XmlRpcValue();
            var subscribeStats = new XmlRpcValue();
            var serviceStats = new XmlRpcValue();

            lock (gate)
            {
                int pidx = 0;
                publishStats.SetArray(advertisedTopics.Count);
                foreach (Publication p in advertisedTopics)
                {
                    publishStats.Set(pidx++, p.GetStats());
                }

                int sidx = 0;
                subscribeStats.SetArray(subscriptions.Count);
                foreach (Subscription s in subscriptions)
                {
                    subscribeStats.Set(sidx++, s.GetStats());
                }
            }

            // TODO: fix for services
            serviceStats.SetArray(0); //service_stats.Size = 0;

            var stats = new XmlRpcValue();
            stats.Set(0, publishStats);
            stats.Set(1, subscribeStats);
            stats.Set(2, serviceStats);
            return stats;
        }

        public XmlRpcValue GetBusInfo()
        {
            var info = new XmlRpcValue();
            info.SetArray(0);
            lock (gate)
            {
                foreach (Publication t in advertisedTopics)
                {
                    t.GetInfo(info);
                }

                foreach (Subscription t in subscriptions)
                {
                    t.GetInfo(info);
                }
            }
            return info;
        }

        public void GetSubscriptions(XmlRpcValue subs)
        {
            subs.SetArray(0);
            lock (gate)
            {
                int i = 0;
                foreach (Subscription t in subscriptions)
                {
                    subs.Set(i++, new XmlRpcValue(t.Name, t.DataType));
                }
            }
        }

        public void GetPublications(XmlRpcValue pubs)
        {
            pubs.SetArray(0);
            lock (gate)
            {
                int i = 0;
                foreach (Publication t in advertisedTopics)
                {
                    XmlRpcValue pub = new XmlRpcValue();
                    pub.Set(0, t.Name);
                    pub.Set(1, t.DataType);
                    pubs.Set(i++, pub);
                }
            }
        }

        public async Task<bool> PubUpdate(string topic, List<string> pubs)
        {
            using (this.logger.BeginScope(nameof(PubUpdate)))
            {
                this.logger.LogDebug("TopicManager is updating publishers for " + topic);

                Subscription sub = null;
                lock (gate)
                {
                    if (shuttingDown)
                        return false;

                    foreach (Subscription s in subscriptions)
                    {
                        if (s.Name != topic || s.IsDisposed)
                            continue;
                        sub = s;
                        break;
                    }
                }

                if (sub != null)
                {
                    return await sub.PubUpdate(pubs);
                }

                this.logger.LogInformation($"Request for updating publishers of topic '{topic}', which has no subscribers.");
                return false;
            }
        }

        private void PublisherUpdateCallback(XmlRpcValue parm, XmlRpcValue result)
        {
            var pubs = new List<string>();
            for (int idx = 0; idx < parm[2].Count; idx++)
                pubs.Add(parm[2][idx].GetString());
            var pubUpdateTask = PubUpdate(parm[1].GetString(), pubs);
            pubUpdateTask.WhenCompleted().WhenCompleted().Wait();
            if (pubUpdateTask.IsCompletedSuccessfully && pubUpdateTask.Result)
                XmlRpcManager.ResponseInt(1, "", 0)(result);
            else
            {
                const string error = "Unknown error while handling XmlRpc call to pubUpdate";
                this.logger.LogError(error);
                XmlRpcManager.ResponseInt(0, error, 0)(result);
            }
        }

        private void RequestTopicCallback(XmlRpcValue parm, XmlRpcValue res)
        {
            //XmlRpcValue res = XmlRpcValue.Create(ref result)
            //	, parm = XmlRpcValue.Create(ref parms);
            //result = res.instance;
            if (!RequestTopic(parm[1].GetString(), parm[2], ref res))
            {
                const string error = "Unknown error while handling XmlRpc call to requestTopic";
                this.logger.LogError(error);
                XmlRpcManager.ResponseInt(0, error, 0)(res);
            }
        }

        private void GetBusStatsCallback(XmlRpcValue parm, XmlRpcValue res)
        {
            res.Set(0, 1);
            res.Set(1, "");
            var response = GetBusStats();
            res.Set(2, response);
        }

        private void GetBusInfoCallback(XmlRpcValue parm, XmlRpcValue res)
        {
            res.Set(0, 1);
            res.Set(1, "");
            var response = GetBusInfo();
            res.Set(2, response);
        }

        private void GetSubscriptionsCallback(XmlRpcValue parm, XmlRpcValue res)
        {
            res.Set(0, 1);
            res.Set(1, "subscriptions");
            var response = new XmlRpcValue();
            GetSubscriptions(response);
            res.Set(2, response);
        }

        private void GetPublicationsCallback(XmlRpcValue parm, XmlRpcValue res)
        {
            res.Set(0, 1);
            res.Set(1, "publications");
            var response = new XmlRpcValue();
            GetPublications(response);
            res.Set(2, response);
        }

        public async Task<bool> Unadvertise(string topic, SubscriberCallbacks callbacks)
        {
            Publication pub = null;

            lock (gate)
            {
                foreach (Publication p in advertisedTopics)
                {
                    if (p.Name == topic && !p.Dropped)
                    {
                        pub = p;
                        break;
                    }
                }
            }

            if (pub == null)
                return false;

            pub.RemoveCallbacks(callbacks);

            lock (gate)
            {
                if (pub.NumCallbacks > 0)
                    return true;
                pub.Dispose();
                advertisedTopics.Remove(pub);
            }

            await UnregisterPublisher(pub.Name);

            return true;
        }
    }
}
